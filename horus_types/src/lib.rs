//! # HORUS Universal Types
//!
//! IPC-optimized message types used by ALL horus verticals — math primitives,
//! diagnostics, time synchronization, and generic cross-language messages.
//!
//! These types are `#[repr(C)]`, Pod-safe for zero-copy shared memory transfer.

/// Register a `#[repr(C)]` struct as a HORUS POD message, with its layout
/// identity.
///
/// # Why the fields are written out here
///
/// The three copies of this macro this replaces took only type names, so a
/// registered type got `Pod`, `Zeroable` and `PodMessage` and nothing else. The
/// consequence was a hole in the one safety net that covers cross-language
/// messages. `Topic::new_checked` compares a `LAYOUT_HASH` before two processes
/// agree to share bytes, and the [`message!`](horus_core::message) macro emits
/// one for every type a *user* declares — but every type HORUS itself ships had
/// none. `Twist` published from C++ and read from Rust, the exact case the
/// product is sold on, had no layout check at all: two builds whose `linear`
/// and `angular` had been swapped opened the same topic without complaint and
/// swapped the velocities on the way through.
///
/// The hash is FNV-1a over `Name|field:type|field:type`, byte-identical to what
/// `message!` emits and to what `horus msg hash` prints, so the number a
/// developer sees in a mismatch error is the number the CLI reports.
///
/// # Why a hand-written field list cannot drift
///
/// A second copy of a struct's fields is exactly the thing that produced
/// `JointCommand` at 928 bytes in Rust against 88 in C++. So the expansion
/// below does not trust this list: it checks, at compile time, that every named
/// field exists with the written type, that each one sits at the `repr(C)`
/// offset the list implies, and that the struct is the size the list implies.
/// Rename, retype, reorder, add or remove a field in the struct without
/// updating the list and the crate does not build.
macro_rules! impl_pod_message {
    ($( $name:ident { $( $field:ident : $fty:ty ),+ $(,)? } )+) => {
        $(
            // SAFETY: every type registered here is `#[repr(C)]` and every
            // field is a primitive, a fixed array of them, or another type
            // registered here — there is no pointer, reference or heap-owning
            // field, which is what the shared-memory transport requires: it
            // reads and writes messages with a raw `ptr::read`/`ptr::write`.
            // The `const _` block below fails the build if that stops being
            // true of the field list.
            unsafe impl horus_core::bytemuck::Pod for $name {}
            unsafe impl horus_core::bytemuck::Zeroable for $name {}
            unsafe impl horus_core::communication::PodMessage for $name {}

            impl $name {
                /// The exact string this type's layout hash is computed over.
                ///
                /// `Name|field:type|field:type`, the same shape `message!`
                /// concatenates and the same shape `horus msg hash` builds from
                /// parsed source, so all three produce one number for one type.
                pub const LAYOUT_CANONICAL: &'static str = concat!(
                    stringify!($name),
                    $( "|", stringify!($field), ":", stringify!($fty), )+
                );

                /// FNV-1a of [`Self::LAYOUT_CANONICAL`] — the number
                /// `horus msg hash` prints and `Topic::new_checked` compares.
                ///
                /// Covers every field's name and written type, so reordering,
                /// renaming, retyping, adding or removing one changes it. The
                /// topic open path on its own checks only the type's short name
                /// and its size, and neither describes layout.
                pub const LAYOUT_HASH: u32 =
                    horus_core::communication::topic::const_fnv1a(
                        Self::LAYOUT_CANONICAL.as_bytes(),
                    );

                /// Open a topic for this message with layout checking on.
                ///
                /// Equivalent to `Topic::new_checked(name, Self::LAYOUT_HASH)`,
                /// and the same helper `message!` generates for hand-written
                /// types. Prefer it over `Topic::<Self>::new(name)`: it costs
                /// nothing at runtime and turns a silently-misread message —
                /// same name, same size, reordered fields — into an error.
                pub fn topic(
                    name: &str,
                ) -> horus_core::error::HorusResult<horus_core::communication::Topic<Self>> {
                    horus_core::communication::Topic::new_checked(name, Self::LAYOUT_HASH)
                }
            }

            // The field list above is a second copy of the struct's fields, and
            // a second copy that is allowed to drift is worse than none: it
            // would make `LAYOUT_HASH` describe a layout the type does not
            // have, so two peers with genuinely different layouts would agree.
            // These checks are what stops that.
            const _: () = {
                // Names and types, exactly. A renamed field stops compiling
                // here; a retyped one does too, because the binding is
                // annotated rather than inferred.
                #[allow(dead_code)]
                fn fields_are_as_declared(v: &$name) {
                    $( let _: &$fty = &v.$field; )+
                }

                const fn round_up(n: usize, align: usize) -> usize {
                    (n + align - 1) & !(align - 1)
                }

                // Offsets, recomputed from the declared order with the
                // `repr(C)` rule, and compared against what the compiler
                // actually laid out. This is the check that catches a
                // reordering — the failure the layout hash exists for.
                let mut cursor = 0usize;
                let mut max_align = 1usize;
                $(
                    let align = core::mem::align_of::<$fty>();
                    cursor = round_up(cursor, align);
                    assert!(
                        core::mem::offset_of!($name, $field) == cursor,
                        concat!(
                            stringify!($name), ".", stringify!($field),
                            " is not where impl_pod_message! says it is — the struct's \
                             fields were reordered without updating the list, so \
                             LAYOUT_HASH would describe a layout this type does not have"
                        )
                    );
                    cursor += core::mem::size_of::<$fty>();
                    if align > max_align {
                        max_align = align;
                    }
                )+
                assert!(
                    core::mem::size_of::<$name>() == round_up(cursor, max_align),
                    concat!(
                        stringify!($name),
                        " is not the size impl_pod_message!'s field list implies — a \
                         field was added or removed without updating the list"
                    )
                );
            };
        )+
    };
}

pub mod diagnostics;
pub mod generic;
pub mod math;
pub mod time;

pub mod prelude;

// Re-export all types at crate root for convenience
pub use diagnostics::*;
pub use generic::{GenericMessage, MAX_GENERIC_PAYLOAD};
pub use math::*;
pub use time::*;

#[cfg(test)]
mod layout_identity_tests {
    use crate::*;

    /// Every registered type, its canonical form and its hash, written down.
    ///
    /// A hash that changes is a fleet-wide event: a robot running the old
    /// build and a laptop running the new one stop being able to open the same
    /// topic, and the two builds that *can* still open it are the ones whose
    /// layouts agree. That is the intent — but it must be a decision, not an
    /// accident, so the numbers are pinned here and a change to any built-in
    /// message has to come through this list.
    #[allow(clippy::type_complexity)]
    fn pinned() -> Vec<(&'static str, u32, &'static str, u32, &'static str)> {
        vec![
            ("Twist", Twist::LAYOUT_HASH, Twist::LAYOUT_CANONICAL, 0xf774_10dc, "Twist|linear:[f64; 3]|angular:[f64; 3]|timestamp_ns:u64"),
            ("Pose2D", Pose2D::LAYOUT_HASH, Pose2D::LAYOUT_CANONICAL, 0x2b8d_c074, "Pose2D|x:f64|y:f64|theta:f64|timestamp_ns:u64"),
            ("TransformStamped", TransformStamped::LAYOUT_HASH, TransformStamped::LAYOUT_CANONICAL, 0x35d8_0d86, "TransformStamped|translation:[f64; 3]|rotation:[f64; 4]|timestamp_ns:u64"),
            ("Point3", Point3::LAYOUT_HASH, Point3::LAYOUT_CANONICAL, 0x6ab9_12d1, "Point3|x:f64|y:f64|z:f64"),
            ("Vector3", Vector3::LAYOUT_HASH, Vector3::LAYOUT_CANONICAL, 0x5c4a_28b8, "Vector3|x:f64|y:f64|z:f64"),
            ("Quaternion", Quaternion::LAYOUT_HASH, Quaternion::LAYOUT_CANONICAL, 0x0e67_4729, "Quaternion|x:f64|y:f64|z:f64|w:f64"),
            ("Pose3D", Pose3D::LAYOUT_HASH, Pose3D::LAYOUT_CANONICAL, 0x5ce1_9b02, "Pose3D|position:Point3|orientation:Quaternion|timestamp_ns:u64"),
            ("PoseStamped", PoseStamped::LAYOUT_HASH, PoseStamped::LAYOUT_CANONICAL, 0x847c_c840, "PoseStamped|pose:Pose3D|frame_id:[u8; 32]|timestamp_ns:u64"),
            ("PoseWithCovariance", PoseWithCovariance::LAYOUT_HASH, PoseWithCovariance::LAYOUT_CANONICAL, 0xa0e7_d274, "PoseWithCovariance|pose:Pose3D|covariance:[f64; 36]|frame_id:[u8; 32]|timestamp_ns:u64"),
            ("TwistWithCovariance", TwistWithCovariance::LAYOUT_HASH, TwistWithCovariance::LAYOUT_CANONICAL, 0x23fa_51e9, "TwistWithCovariance|twist:Twist|covariance:[f64; 36]|frame_id:[u8; 32]|timestamp_ns:u64"),
            ("Accel", Accel::LAYOUT_HASH, Accel::LAYOUT_CANONICAL, 0xed9a_a353, "Accel|linear:[f64; 3]|angular:[f64; 3]|timestamp_ns:u64"),
            ("AccelStamped", AccelStamped::LAYOUT_HASH, AccelStamped::LAYOUT_CANONICAL, 0x54e2_467c, "AccelStamped|accel:Accel|frame_id:[u8; 32]|timestamp_ns:u64"),
            ("Heartbeat", Heartbeat::LAYOUT_HASH, Heartbeat::LAYOUT_CANONICAL, 0xcd6b_f181, "Heartbeat|node_name:[u8; 32]|node_id:u32|sequence:u64|alive:u8|uptime:f64|timestamp_ns:u64"),
            ("DiagnosticStatus", DiagnosticStatus::LAYOUT_HASH, DiagnosticStatus::LAYOUT_CANONICAL, 0x0760_0c13, "DiagnosticStatus|level:u8|code:u32|message:[u8; 128]|component:[u8; 32]|timestamp_ns:u64"),
            ("EmergencyStop", EmergencyStop::LAYOUT_HASH, EmergencyStop::LAYOUT_CANONICAL, 0x6703_5653, "EmergencyStop|engaged:u8|reason:[u8; 64]|source:[u8; 32]|auto_reset:u8|timestamp_ns:u64"),
            ("ResourceUsage", ResourceUsage::LAYOUT_HASH, ResourceUsage::LAYOUT_CANONICAL, 0x0610_5669, "ResourceUsage|cpu_percent:f32|memory_bytes:u64|memory_percent:f32|disk_bytes:u64|disk_percent:f32|network_tx_bytes:u64|network_rx_bytes:u64|temperature:f32|thread_count:u32|timestamp_ns:u64"),
            ("DiagnosticValue", DiagnosticValue::LAYOUT_HASH, DiagnosticValue::LAYOUT_CANONICAL, 0x8df2_60d9, "DiagnosticValue|key:[u8; 32]|value:[u8; 64]|value_type:u8"),
            ("DiagnosticReport", DiagnosticReport::LAYOUT_HASH, DiagnosticReport::LAYOUT_CANONICAL, 0x1952_64ee, "DiagnosticReport|component:[u8; 32]|values:[DiagnosticValue; 16]|value_count:u8|level:u8|timestamp_ns:u64"),
            ("NodeHeartbeat", NodeHeartbeat::LAYOUT_HASH, NodeHeartbeat::LAYOUT_CANONICAL, 0x41de_5fd5, "NodeHeartbeat|state:u8|health:u8|tick_count:u64|target_rate_hz:u32|actual_rate_hz:u32|error_count:u32|last_tick_timestamp:u64|heartbeat_timestamp:u64"),
            ("SafetyStatus", SafetyStatus::LAYOUT_HASH, SafetyStatus::LAYOUT_CANONICAL, 0x66c4_578c, "SafetyStatus|enabled:u8|estop_engaged:u8|watchdog_ok:u8|limits_ok:u8|comms_ok:u8|mode:u8|fault_code:u32|timestamp_ns:u64"),
            ("Clock", Clock::LAYOUT_HASH, Clock::LAYOUT_CANONICAL, 0x044c_9261, "Clock|clock_ns:u64|realtime_ns:u64|sim_speed:f64|paused:u8|source:u8|_pad:[u8; 6]|timestamp_ns:u64"),
            ("TimeReference", TimeReference::LAYOUT_HASH, TimeReference::LAYOUT_CANONICAL, 0xc7c9_4e2e, "TimeReference|time_ref_ns:u64|source:[u8; 32]|offset_ns:i64|timestamp_ns:u64"),
            ("SimSync", SimSync::LAYOUT_HASH, SimSync::LAYOUT_CANONICAL, 0x6d28_9920, "SimSync|step:u64|sim_time_ns:u64|dt_ns:u64|state:u8|_pad:[u8; 7]"),
            ("RateRequest", RateRequest::LAYOUT_HASH, RateRequest::LAYOUT_CANONICAL, 0x5178_1a4b, "RateRequest|topic_name:[u8; 32]|desired_hz:f64|min_hz:f64|max_hz:f64|requester_id:u64|timestamp_ns:u64"),
        ]
    }

    /// FNV-1a, written out rather than imported, so a change to
    /// `const_fnv1a` has to be made here too and cannot pass unnoticed.
    fn fnv1a(bytes: &[u8]) -> u32 {
        let mut hash: u32 = 2166136261;
        for &byte in bytes {
            hash ^= byte as u32;
            hash = hash.wrapping_mul(16777619);
        }
        hash
    }

    #[test]
    fn every_builtin_has_the_layout_identity_it_had_yesterday() {
        for (name, hash, canonical, want_hash, want_canonical) in pinned() {
            assert_eq!(
                canonical, want_canonical,
                "`{name}`'s field list changed. That is a wire-format change: \
                 every peer built against the old one stops matching."
            );
            assert_eq!(
                hash, want_hash,
                "`{name}`'s layout hash changed without its canonical form changing, \
                 which can only mean the hash function did"
            );
        }
    }

    /// The hash the macro computes must be FNV-1a of the string it published,
    /// not of some other string.
    #[test]
    fn the_hash_is_fnv1a_of_the_published_canonical_form() {
        for (name, hash, canonical, _, _) in pinned() {
            assert_eq!(
                hash,
                fnv1a(canonical.as_bytes()),
                "`{name}` publishes a canonical form its own hash was not computed from"
            );
        }
    }

    /// The property the whole mechanism exists for, on a shipped type.
    ///
    /// `Twist` is the case in the cross-language tutorial. Two builds that had
    /// swapped `linear` and `angular` would produce the same short type name
    /// and the same 56 bytes, which is all the topic open path checks on its
    /// own.
    #[test]
    fn reordering_a_builtin_changes_its_hash() {
        assert_ne!(
            fnv1a(Twist::LAYOUT_CANONICAL.as_bytes()),
            fnv1a(b"Twist|angular:[f64; 3]|linear:[f64; 3]|timestamp_ns:u64"),
        );
        assert_eq!(
            core::mem::size_of::<Twist>(),
            56,
            "the two orderings are the same size, which is why the size check \
             cannot tell them apart"
        );
    }

    /// A built-in must be openable with the check on. Before
    /// `impl_pod_message!` emitted these, the only way to open a topic for a
    /// shipped type was `Topic::<Twist>::new`, which binds hash 0 and disables
    /// the check — so the types HORUS ships were the least protected ones in
    /// the system.
    #[test]
    fn a_builtin_topic_rejects_a_different_layout() {
        let name = format!("layout_check_probe_{}", std::process::id());
        let ok = Twist::topic(&name);
        assert!(
            ok.is_ok(),
            "opening with the real hash must work: {:?}",
            ok.err()
        );

        // A second opener claiming a different layout for the same topic. This
        // is the reordered-fields build, arriving from C++ or from an older
        // Rust binary.
        let clash = horus_core::communication::Topic::<Twist>::new_checked(
            &name,
            Twist::LAYOUT_HASH ^ 0x5555_5555,
        );
        assert!(
            clash.is_err(),
            "a peer declaring a different layout for `{name}` was accepted"
        );
    }
}
