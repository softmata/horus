//! Contract tests over the hand-written Python surface in `horus/__init__.py`.
//!
//! Nothing here is compiled into the extension module — the module is declared
//! `#[cfg(test)]` in `lib.rs`. `Node` is pure Python and never reaches this
//! crate, but its docstrings are the only API reference a user gets at the
//! prompt, so they are worth pinning the same way `horus_manager` pins its
//! README and template: read the file from disk at test time and assert on it,
//! so whichever side drifts is the side that fails.
//!
//! The defect these were written against: `Node.__init__`'s "Accepts:" list for
//! `pubs` enumerated only the list and dict spellings, while the parameter's own
//! type annotation and `Node._parse_topics` had always taken a bare `str` too.
//! `help(horus.Node)` renders that docstring and nothing else — `_parse_topics`
//! starts with an underscore, so pydoc hides it — which left `pubs="motors.cmd_vel"`
//! (the spelling the project template itself used to generate) undiscoverable
//! without reading the binding source. That was the original complaint: "whether
//! both forms are accepted is undiscoverable".

use std::fs;
use std::path::PathBuf;

/// `horus/__init__.py`, the hand-written half of the `horus` package.
fn init_py() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("horus")
        .join("__init__.py");
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

/// The body of `class Node:` — from its `class` line to the next top-level item.
fn node_class_body(src: &str) -> &str {
    let start = src
        .find("\nclass Node:")
        .expect("horus/__init__.py should define `class Node:` at module level");
    let rest = &src[start + 1..];
    match rest[1..].find("\nclass ") {
        Some(offset) => &rest[..offset + 1],
        None => rest,
    }
}

/// The docstring of `def <name>` inside a class body, without its quotes.
fn method_docstring(class_body: &str, name: &str) -> String {
    let needle = format!("\n    def {name}(");
    let at = class_body
        .find(&needle)
        .unwrap_or_else(|| panic!("Node should define `def {name}(`"));
    let after_def = &class_body[at..];
    let open = after_def
        .find("\"\"\"")
        .unwrap_or_else(|| panic!("`def {name}(` should be followed by a docstring"));
    let body = &after_def[open + 3..];
    let close = body
        .find("\"\"\"")
        .unwrap_or_else(|| panic!("`def {name}(`'s docstring should be closed"));
    body[..close].to_string()
}

/// One entry of a Google-style `Args:` block: the `<arg>:` line plus every
/// continuation line indented under it.
fn arg_entry(docstring: &str, arg: &str) -> String {
    let marker = format!("{arg}:");
    let mut lines = docstring.lines();
    let first = lines
        .find(|line| line.trim_start().starts_with(&marker))
        .unwrap_or_else(|| panic!("docstring should document the `{arg}` argument"));
    let base_indent = first.len() - first.trim_start().len();

    let mut entry = String::from(first);
    for line in lines {
        let indent = line.len() - line.trim_start().len();
        if !line.trim().is_empty() && indent <= base_indent {
            break;
        }
        entry.push('\n');
        entry.push_str(line);
    }
    entry
}

/// The ``…`` code spans in a chunk of reStructuredText docstring.
fn code_spans(text: &str) -> Vec<String> {
    text.split("``")
        .skip(1)
        .step_by(2)
        .map(|span| span.trim().to_string())
        .collect()
}

/// Is this code span a single bare topic string — `"data"` rather than
/// `["data"]` or `{"cmd": CmdVel}`?
fn is_bare_topic_string(span: &str) -> bool {
    let inner = match span.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
        Some(inner) => inner,
        None => return false,
    };
    !inner.is_empty() && !inner.contains(['"', '[', ']', '{', '}', ':'])
}

/// The annotation of one parameter of `Node.__init__`, read off the signature.
fn init_annotation(class_body: &str, param: &str) -> String {
    let at = class_body
        .find("\n    def __init__(")
        .expect("Node should define `def __init__(`");
    let signature_end = class_body[at..]
        .find("\"\"\"")
        .expect("`def __init__(` should be followed by a docstring");
    let signature = &class_body[at..at + signature_end];

    let marker = format!("{param}:");
    let line = signature
        .lines()
        .find(|line| line.trim_start().starts_with(&marker))
        .unwrap_or_else(|| panic!("`Node.__init__` should take a `{param}` parameter"));
    line.trim_start()
        .trim_start_matches(&marker)
        .trim()
        .trim_end_matches(',')
        .trim()
        .to_string()
}

/// `help(horus.Node)` shows `__init__`'s docstring and nothing else, so the
/// "Accepts:" list under `pubs` is the one place a user can find out that
/// `pubs="motors.cmd_vel"` is legal. It used to stop at the list and dict
/// forms.
#[test]
fn node_init_docstring_lists_the_bare_string_topic_form() {
    let src = init_py();
    let body = node_class_body(&src);
    let doc = method_docstring(body, "__init__");

    for arg in ["pubs", "subs"] {
        let entry = arg_entry(&doc, arg);
        let spans = code_spans(&entry);
        assert!(
            spans.iter().any(|span| is_bare_topic_string(span)),
            "`Node.__init__`'s docstring for `{arg}` should show the bare string \
             form (e.g. ``\"data\"``) alongside the list and dict forms — the \
             annotation and `_parse_topics` both accept it, and this docstring is \
             all `help(horus.Node)` shows. Code spans found: {spans:?}\n\
             ---- entry ----\n{entry}"
        );
    }
}

/// The sibling enumeration: `_parse_topics` carries its own "Accepts:" table,
/// and it listed neither `Sub` nor `Pub` even though the function branches on
/// them and the `Sub`/`Pub` class docstrings tell users to pass them.
#[test]
fn parse_topics_docstring_covers_every_form_it_branches_on() {
    let src = init_py();
    let body = node_class_body(&src);
    let doc = method_docstring(body, "_parse_topics");

    for form in ["\"topic\"", "Sub(", "Pub(", "{\"cmd\": CmdVel}"] {
        assert!(
            doc.contains(form),
            "`_parse_topics`'s Accepts table should list the `{form}` form it \
             handles.\n---- table ----\n{doc}"
        );
    }
}

/// The type annotation is the editor's answer to the same question, and
/// `horus/py.typed` ships, so a type checker reads it. It said
/// `Optional[Union[List[str], str, Dict[str, Dict]]]`, which rejects
/// `subs=[horus.Sub("panda.joint_state", horus.JointState)]` — the exact call
/// the `Sub` docstring shows. mypy: `List item 0 has incompatible type "Sub";
/// expected "str"`.
#[test]
fn pubs_and_subs_annotations_admit_every_documented_form() {
    let src = init_py();
    let body = node_class_body(&src);

    for param in ["pubs", "subs"] {
        let annotation = init_annotation(body, param);
        for form in ["str", "Sub", "Pub", "Dict"] {
            assert!(
                annotation.contains(form),
                "`Node.__init__`'s `{param}` annotation should admit `{form}` — \
                 `_parse_topics` accepts it and the docstrings recommend it. \
                 Annotation is `{annotation}`"
            );
        }
    }
}
