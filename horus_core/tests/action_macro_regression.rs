// Regression test for action! macro with 3+ result fields.
//
// The action! macro previously failed with "expected token: ," when the
// result section had 3 or more fields. The root cause was nested macro
// invocation: $crate::action!(@fmt_args ...) inside paste! { ... } broke
// when the token count exceeded a threshold. Fixed by inlining the
// LogSummary format string generation instead of delegating to @fmt_args.

use horus_core::action;
use horus_core::core::LogSummary;

// This previously failed with "expected token: ," when result had 3+ fields
action! {
    BehTestNav {
        goal {
            target_x: f64,
            target_y: f64,
        }
        feedback {
            progress: f32,
        }
        result {
            success: bool,
            final_x: f64,
            final_y: f64,
        }
    }
}

// Even more fields — standard_action uses 6 result fields
action! {
    BigAction {
        goal {
            a: f64,
            b: f64,
            c: f64,
            d: f64,
        }
        feedback {
            p1: f32,
            p2: f32,
            p3: f32,
        }
        result {
            ok: bool,
            x: f64,
            y: f64,
            z: f64,
            w: f64,
            elapsed: f32,
        }
    }
}

#[test]
fn test_3_result_fields() {
    let r = BehTestNavResult {
        success: true,
        final_x: 10.0,
        final_y: 20.0,
    };
    assert!(r.success);
    assert_eq!(r.final_x, 10.0);
    assert_eq!(r.final_y, 20.0);
}

#[test]
fn test_6_result_fields() {
    let r = BigActionResult {
        ok: true,
        x: 1.0,
        y: 2.0,
        z: 3.0,
        w: 4.0,
        elapsed: 5.0,
    };
    assert!(r.ok);
}

#[test]
fn test_log_summary_3_fields() {
    let r = BehTestNavResult {
        success: true,
        final_x: 10.0,
        final_y: 20.0,
    };
    let summary = r.log_summary();
    assert!(summary.contains("success"));
    assert!(summary.contains("final_x"));
    assert!(summary.contains("final_y"));
}

#[test]
fn test_log_summary_6_fields() {
    let r = BigActionResult {
        ok: true,
        x: 1.0,
        y: 2.0,
        z: 3.0,
        w: 4.0,
        elapsed: 5.0,
    };
    let summary = r.log_summary();
    assert!(summary.contains("ok"));
    assert!(summary.contains("elapsed"));
}

#[test]
fn test_4_goal_fields() {
    let g = BigActionGoal {
        a: 1.0,
        b: 2.0,
        c: 3.0,
        d: 4.0,
    };
    let summary = g.log_summary();
    assert!(summary.contains("a="));
    assert!(summary.contains("d="));
}
