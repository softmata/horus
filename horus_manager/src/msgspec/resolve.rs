//! Resolving type references, and ordering messages so nesting works.
//!
//! The parser cannot resolve a reference on its own: `wind: Vec3` is a
//! reference to a type that may be declared in another file, so the name is
//! recorded and the paths are left empty until every message in the package is
//! known. Nothing filled them in. [`super::canonical::render_rust`] and
//! [`render_cpp`](super::canonical::render_cpp) return
//! `Resolved::rust_path`/`cpp_path` verbatim, so an empty path went straight
//! into the artifacts:
//!
//! ```text
//! pub struct WeatherData {
//!     pub temperature: f32,
//!     pub wind: ,                      // <- not Rust
//!     pub ts: u64,
//! }
//! pub const LAYOUT_CANONICAL: &str = "WeatherData|temperature:f32|wind:|ts:u64";
//! ```
//!
//! and `horus msg gen` printed `Generated 2 message(s)` over the top of it. The
//! truncated canonical form is the worse half: it is the layout identity, and
//! `WeatherData { wind: Vec3 }` and `WeatherData { wind: Quat }` produce the
//! same string and therefore the same hash — a layout check that cannot see the
//! field it is checking.
//!
//! # Why the order changes here too
//!
//! C++ and Python need a type declared before it is used: a `struct` member and
//! a `ctypes` `_fields_` entry both name a type that must already exist. The
//! generator emitted messages in the order it read them — files sorted by name,
//! then declaration order within each file — so `Vec3` in `msgs/vec3.hmsg` was
//! emitted *after* `WeatherData` in `msgs/weather.hmsg`, and the only advice
//! the error could give ("declare it above") was impossible to follow across
//! two files. Sorting by dependency instead makes file names irrelevant, which
//! is what a user expects of a definition directory.

use std::collections::HashMap;

use super::{Diag, MsgDef, Package, Type};

/// Built-in types a user is likely to reach for, and the fields to write
/// instead.
///
/// [`super::layout`] deliberately carries no table of built-in *layouts* — the
/// first version did, with `Twist` written down as 48 bytes when it is 56, and
/// a wrong size produces a header whose `static_assert` passes against the
/// wrong number. That reasoning is about sizes this module must not invent. It
/// does not stop the diagnostic from naming the fields: they are checked
/// against `horus_types` by `builtin_hints_match_horus_types` below, and if
/// they drift the test fails rather than the robot.
const BUILTIN_HINTS: &[(&str, &[(&str, &str)])] = &[
    ("Vector3", &[("x", "f64"), ("y", "f64"), ("z", "f64")]),
    ("Point3", &[("x", "f64"), ("y", "f64"), ("z", "f64")]),
    (
        "Quaternion",
        &[("x", "f64"), ("y", "f64"), ("z", "f64"), ("w", "f64")],
    ),
    (
        "Pose2D",
        &[
            ("x", "f64"),
            ("y", "f64"),
            ("theta", "f64"),
            ("timestamp_ns", "u64"),
        ],
    ),
    (
        "Twist",
        &[
            ("linear", "[f64; 3]"),
            ("angular", "[f64; 3]"),
            ("timestamp_ns", "u64"),
        ],
    ),
    (
        "Accel",
        &[
            ("linear", "[f64; 3]"),
            ("angular", "[f64; 3]"),
            ("timestamp_ns", "u64"),
        ],
    ),
];

/// The hint's fields, spelled the way they would be written in a `.hmsg`.
fn hint_fields(fields: &[(&str, &str)]) -> String {
    fields
        .iter()
        .map(|(n, t)| format!("{n}: {t}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Fill in every reference's paths and order the messages by dependency.
///
/// Returns every problem it can find rather than the first, so one run of
/// `horus msg gen` names every unknown type.
pub fn resolve(pkg: &mut Package) -> Result<(), Vec<Diag>> {
    let known: HashMap<String, usize> = pkg
        .messages
        .iter()
        .enumerate()
        .map(|(i, m)| (m.name.clone(), i))
        .collect();

    let mut errors = Vec::new();

    for m in &mut pkg.messages {
        let file = m.src.clone();
        for f in &mut m.fields {
            let (line, col) = (f.line, f.col);
            walk_mut(&mut f.ty, &mut |r| {
                if known.contains_key(&r.short) {
                    // Everything a `.hmsg` set produces lands in one Rust
                    // module and one C++ namespace, so the short name is the
                    // path in both.
                    r.rust_path.clone_from(&r.short);
                    r.cpp_path.clone_from(&r.short);
                    r.external = false;
                    return;
                }
                let mut d = Diag::new(&file, line, col, format!("unknown type `{}`", r.short));
                d = match BUILTIN_HINTS.iter().find(|(n, _)| *n == r.short) {
                    Some((n, fields)) => d.with_help(format!(
                        "`{n}` is a built-in HORUS type, and `horus msg gen` sizes only what \
                         it can see. Write the fields out — `{}` — or declare a message \
                         of your own with them.",
                        hint_fields(fields)
                    )),
                    None => d.with_help(
                        "a message may use primitives (u8..u64, i8..i64, f32, f64, bool), \
                         fixed arrays of them, and other messages declared in msgs/. \
                         Check the spelling, or add a `.hmsg` for it.",
                    ),
                };
                errors.push(d);
            });
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    match dependency_order(&pkg.messages, &known) {
        Ok(order) => {
            let mut taken: Vec<Option<MsgDef>> = pkg.messages.drain(..).map(Some).collect();
            pkg.messages = order.into_iter().filter_map(|i| taken[i].take()).collect();
            Ok(())
        }
        Err(diags) => Err(diags),
    }
}

/// Declaration indices, dependencies first.
///
/// Stable: a message with no unmet dependency keeps its declared position
/// relative to its peers, so the generated files do not shuffle between runs
/// and `--check` stays meaningful.
fn dependency_order(
    messages: &[MsgDef],
    known: &HashMap<String, usize>,
) -> Result<Vec<usize>, Vec<Diag>> {
    #[derive(Clone, Copy, PartialEq)]
    enum Mark {
        New,
        Active,
        Done,
    }

    let mut mark = vec![Mark::New; messages.len()];
    let mut out = Vec::with_capacity(messages.len());
    let mut errors = Vec::new();
    // Explicit stack rather than recursion: a hand-written cycle is short, but
    // a generator feeding this deserves not to blow the stack.
    let mut stack: Vec<(usize, usize)> = Vec::new();

    for root in 0..messages.len() {
        if mark[root] != Mark::New {
            continue;
        }
        stack.push((root, 0));
        mark[root] = Mark::Active;
        while let Some(&mut (node, ref mut next)) = stack.last_mut() {
            let deps = dependencies(&messages[node], known);
            if *next < deps.len() {
                let dep = deps[*next];
                *next += 1;
                match mark[dep] {
                    Mark::New => {
                        mark[dep] = Mark::Active;
                        stack.push((dep, 0));
                    }
                    Mark::Active => {
                        let m = &messages[dep];
                        errors.push(
                            Diag::new(
                                &m.src,
                                m.line,
                                1,
                                format!(
                                    "message `{}` contains itself, directly or through `{}`",
                                    m.name, messages[node].name
                                ),
                            )
                            .with_help(
                                "a message is a fixed-size block of bytes, so one that \
                                 contains itself has no size. Store an index or an id \
                                 instead of the message.",
                            ),
                        );
                        // Treat it as satisfied so the walk terminates and the
                        // remaining messages still get checked.
                        mark[dep] = Mark::Done;
                    }
                    Mark::Done => {}
                }
            } else {
                mark[node] = Mark::Done;
                out.push(node);
                stack.pop();
            }
        }
    }

    if errors.is_empty() {
        Ok(out)
    } else {
        Err(errors)
    }
}

/// Indices of the messages `m` embeds, in field order, without duplicates.
fn dependencies(m: &MsgDef, known: &HashMap<String, usize>) -> Vec<usize> {
    let mut deps = Vec::new();
    for f in &m.fields {
        walk(&f.ty, &mut |short| {
            if let Some(&i) = known.get(short) {
                if !deps.contains(&i) {
                    deps.push(i);
                }
            }
        });
    }
    deps
}

fn walk(t: &Type, f: &mut impl FnMut(&str)) {
    match t {
        Type::Prim(_) => {}
        Type::Array(e, _) => walk(e, f),
        Type::Ref(r) => f(&r.short),
    }
}

fn walk_mut(t: &mut Type, f: &mut impl FnMut(&mut super::Resolved)) {
    match t {
        Type::Prim(_) => {}
        Type::Array(e, _) => walk_mut(e, f),
        Type::Ref(r) => f(r),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgspec::{canonical, parse};
    use std::path::PathBuf;

    fn pkg(src: &str) -> Package {
        let path = PathBuf::from("test.hmsg");
        Package {
            name: "demo".into(),
            messages: parse::parse_file(src, &path).expect("parses"),
        }
    }

    /// The defect: an in-package reference kept an empty path, so the type it
    /// named vanished from the struct and from the layout identity.
    #[test]
    fn an_in_package_reference_gets_a_path() {
        let mut p = pkg("Vec3 { x: f64, y: f64, z: f64 }\n\
                         WeatherData { temperature: f32, wind: Vec3, ts: u64 }\n");
        resolve(&mut p).expect("resolves");
        let wd = p
            .messages
            .iter()
            .find(|m| m.name == "WeatherData")
            .expect("WeatherData");
        assert_eq!(
            canonical::render_rust(&wd.fields[1].ty),
            "Vec3",
            "the Rust rendering of a nested reference must name the type"
        );
        assert_eq!(canonical::render_cpp(&wd.fields[1].ty), "Vec3");
        assert_eq!(
            canonical::canonical_form(wd),
            "WeatherData|temperature:f32|wind:Vec3|ts:u64",
            "the layout identity must carry the nested type; without it \
             `wind: Vec3` and `wind: Quat` hash the same"
        );
    }

    /// The same, one array deeper.
    #[test]
    fn a_reference_inside_an_array_gets_a_path() {
        let mut p = pkg("Vec3 { x: f64, y: f64, z: f64 }\n\
                         Path { points: [Vec3; 8] }\n");
        resolve(&mut p).expect("resolves");
        let path = p.messages.iter().find(|m| m.name == "Path").unwrap();
        assert_eq!(canonical::render_rust(&path.fields[0].ty), "[Vec3; 8]");
        assert_eq!(
            canonical::render_cpp(&path.fields[0].ty),
            "std::array<Vec3, 8>"
        );
    }

    /// Two hashes that must differ. Before the resolve pass both canonical
    /// forms were `Holder|inner:`, so the layout check could not tell a
    /// three-f64 payload from a four-f64 one.
    #[test]
    fn two_different_nested_types_do_not_hash_alike() {
        let mut a = pkg("Inner { x: f64, y: f64, z: f64 }\nHolder { inner: Inner }\n");
        let mut b = pkg("Other { x: f64, y: f64, z: f64, w: f64 }\nHolder { inner: Other }\n");
        resolve(&mut a).expect("a");
        resolve(&mut b).expect("b");
        let ha = canonical::layout_hash(a.messages.iter().find(|m| m.name == "Holder").unwrap());
        let hb = canonical::layout_hash(b.messages.iter().find(|m| m.name == "Holder").unwrap());
        assert_ne!(ha, hb, "a nested type must be part of the layout hash");
    }

    /// Dependencies first, whatever the declaration order — C++ and Python
    /// both need the inner type to exist before the outer one names it, and
    /// `msgs/vec3.hmsg` sorts after `msgs/weather.hmsg`.
    #[test]
    fn messages_are_ordered_by_dependency() {
        let mut p = pkg("WeatherData { wind: Vec3 }\nVec3 { x: f64, y: f64, z: f64 }\n");
        resolve(&mut p).expect("resolves");
        let names: Vec<&str> = p.messages.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, vec!["Vec3", "WeatherData"]);
    }

    /// Independent messages keep their declared order, so the generated files
    /// are stable and `horus msg gen --check` means something.
    #[test]
    fn independent_messages_keep_their_declared_order() {
        let mut p = pkg("B { x: f64 }\nA { y: f64 }\nC { z: f64 }\n");
        resolve(&mut p).expect("resolves");
        let names: Vec<&str> = p.messages.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, vec!["B", "A", "C"]);
    }

    #[test]
    fn an_unknown_type_is_reported_with_a_position() {
        let mut p = pkg("Holder { thing: Nonesuch }\n");
        let errs = resolve(&mut p).expect_err("must not resolve");
        assert_eq!(errs.len(), 1);
        assert!(errs[0].message.contains("Nonesuch"), "{:?}", errs[0]);
        assert_eq!(errs[0].line, 1);
        assert!(errs[0].col > 1, "the column must point at the field");
    }

    /// A built-in gets the fields to paste, not just a refusal.
    #[test]
    fn a_builtin_reference_names_the_fields_to_write() {
        let mut p = pkg("Holder { wind: Vector3 }\n");
        let errs = resolve(&mut p).expect_err("must not resolve");
        let help = errs[0].help.as_deref().unwrap_or("");
        assert!(help.contains("x: f64, y: f64, z: f64"), "{help}");
    }

    #[test]
    fn a_self_reference_is_a_diagnostic_not_a_hang() {
        let mut p = pkg("Node { next: Node, value: f64 }\n");
        let errs = resolve(&mut p).expect_err("must not resolve");
        assert!(errs[0].message.contains("contains itself"), "{:?}", errs[0]);
    }

    #[test]
    fn a_two_step_cycle_is_a_diagnostic_not_a_hang() {
        let mut p = pkg("A { b: B }\nB { a: A }\n");
        let errs = resolve(&mut p).expect_err("must not resolve");
        assert!(
            errs.iter().any(|d| d.message.contains("contains itself")),
            "{errs:?}"
        );
    }

    /// The hints are advice about types this crate does not own. If
    /// `horus_types` changes one, the advice becomes wrong — and wrong advice
    /// about a layout is how `JointCommand` reached 928 bytes against 88.
    ///
    /// `LAYOUT_CANONICAL` is the string the runtime layout check is computed
    /// over, so comparing against it compares against the real field list, not
    /// against a second copy of the same guess.
    #[test]
    fn builtin_hints_match_horus_types() {
        let actual: &[(&str, &str)] = &[
            ("Vector3", horus_types::Vector3::LAYOUT_CANONICAL),
            ("Point3", horus_types::Point3::LAYOUT_CANONICAL),
            ("Quaternion", horus_types::Quaternion::LAYOUT_CANONICAL),
            ("Pose2D", horus_types::Pose2D::LAYOUT_CANONICAL),
            ("Twist", horus_types::Twist::LAYOUT_CANONICAL),
            ("Accel", horus_types::Accel::LAYOUT_CANONICAL),
        ];
        assert_eq!(
            actual.len(),
            BUILTIN_HINTS.len(),
            "every hint needs a horus_types counterpart here"
        );
        for (name, fields) in BUILTIN_HINTS {
            let want = format!(
                "{name}{}",
                fields
                    .iter()
                    .map(|(n, t)| format!("|{n}:{t}"))
                    .collect::<String>()
            );
            let have = actual
                .iter()
                .find(|(n, _)| n == name)
                .map(|(_, c)| *c)
                .unwrap_or("");
            assert_eq!(
                want, have,
                "the `.hmsg` hint for `{name}` no longer matches horus_types"
            );
        }
    }
}
