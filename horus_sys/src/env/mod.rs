//! One vocabulary for boolean environment variables.
//!
//! # Why this exists
//!
//! HORUS reads 60-odd `HORUS_*` variables, and before this module every site
//! decided for itself what counted as "on". The forms in production were:
//!
//! ```text
//! v == "true" || v == "1"                            HORUS_LOG_FILE
//! !(v.is_empty() || v == "0" || eq_ic(v, "false"))   HORUS_SIM_MODE
//! v == "0" || eq_ic(v, "false")                      HORUS_NET_ENABLED  (opt-out)
//! env::var(..).is_ok()                               presence-only
//! v == "1"                                           HORUS_WIN_REALTIME_CLASS
//! ```
//!
//! The consequence was not academic. `HORUS_LOG_FILE=yes` was **false** while
//! `HORUS_SIM_MODE=no` was **true** — the second one silently swapping every
//! driver for an inert `SimStubNode`, actuators never commanded and sensors
//! never read. Two variables in the same framework disagreed about the word
//! "no", and nothing in the codebase said which was right.
//!
//! # The vocabulary
//!
//! `1`, `true`, `yes`, `on` are true. `0`, `false`, `no`, `off` and the empty
//! string are false. Matching is ASCII case-insensitive, so `True` and `ON`
//! work. Surrounding whitespace is trimmed, because a value that arrived from a
//! YAML launch file or a shell here-doc often carries some.
//!
//! # Why an unrecognised value is `None`, not `false`
//!
//! `env_flag` returns `Option<bool>`: `None` means "unset or not a boolean I
//! recognise", which leaves the caller's default in force. `HORUS_SIM_MODE=maybe`
//! therefore keeps simulation off rather than turning it on — the old
//! `!(v == "0")` shape treated every typo as an opt-in, which is the wrong
//! direction for a flag that disconnects the robot from its hardware.
//!
//! It also matches what the reference page already promises for
//! `HORUS_SHM_PREFAULT`: "an unrecognised value keeps the default rather than
//! meaning off".

/// Read a boolean environment variable using the project-wide vocabulary.
///
/// Returns `None` when the variable is unset, or set to something that is not
/// a recognised truth value — in both cases the caller's default should stand.
///
/// ```
/// # use horus_sys::env::parse_flag;
/// assert_eq!(parse_flag("yes"),   Some(true));
/// assert_eq!(parse_flag("off"),   Some(false));
/// assert_eq!(parse_flag(""),      Some(false));
/// // Not a truth value: the caller's default stands.
/// assert_eq!(parse_flag("maybe"), None);
/// ```
///
/// The example exercises [`parse_flag`] rather than `env_flag` on purpose: an
/// assertion about a variable being *unset* is only true until someone's shell
/// or CI image happens to set it, and a doctest that depends on the ambient
/// environment is a flake waiting for a machine that disagrees.
pub fn env_flag(name: &str) -> Option<bool> {
    parse_flag(std::env::var(name).ok()?.as_str())
}

/// The parsing half, separated from the environment so it can be tested
/// without mutating process-global state — `set_var` is not safe against a
/// concurrent `getenv` on another thread, and the test binaries here run
/// in parallel.
pub fn parse_flag(raw: &str) -> Option<bool> {
    let v = raw.trim();
    if v.is_empty() {
        return Some(false);
    }
    if v.eq_ignore_ascii_case("1")
        || v.eq_ignore_ascii_case("true")
        || v.eq_ignore_ascii_case("yes")
        || v.eq_ignore_ascii_case("on")
    {
        return Some(true);
    }
    if v.eq_ignore_ascii_case("0")
        || v.eq_ignore_ascii_case("false")
        || v.eq_ignore_ascii_case("no")
        || v.eq_ignore_ascii_case("off")
    {
        return Some(false);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_whole_vocabulary_is_accepted_in_any_case() {
        for t in ["1", "true", "TRUE", "True", "yes", "YES", "on", "On"] {
            assert_eq!(parse_flag(t), Some(true), "{t:?} must be true");
        }
        for f in ["0", "false", "FALSE", "no", "NO", "off", "Off", ""] {
            assert_eq!(parse_flag(f), Some(false), "{f:?} must be false");
        }
    }

    /// The two divergences that motivated this module, pinned as tests.
    #[test]
    fn the_words_that_used_to_disagree_now_agree() {
        // `HORUS_LOG_FILE=yes` was false because that site matched only
        // "true"/"1".
        assert_eq!(parse_flag("yes"), Some(true));
        // `HORUS_SIM_MODE=no` was TRUE, because that site treated anything
        // that was not "0"/"false"/empty as an opt-in — so a deploy script
        // saying "no" got inert SimStubNodes and no diagnostic.
        assert_eq!(parse_flag("no"), Some(false));
    }

    #[test]
    fn an_unrecognised_value_leaves_the_default_in_force() {
        for junk in ["maybe", "2", "yep", "sim", "-1"] {
            assert_eq!(
                parse_flag(junk),
                None,
                "{junk:?} must not be read as a truth value in either direction"
            );
        }
    }

    #[test]
    fn whitespace_from_a_launch_file_or_here_doc_is_trimmed() {
        assert_eq!(parse_flag(" true "), Some(true));
        assert_eq!(parse_flag("\toff\n"), Some(false));
    }

    /// `env_flag` is the thin half — it reads the variable and hands the value
    /// to `parse_flag`. Asserting it on a name this process itself controls
    /// keeps the check honest without depending on what the ambient
    /// environment happens to hold.
    #[test]
    fn env_flag_delegates_to_parse_flag() {
        // A name unique to this process, so a developer's shell or a CI image
        // cannot decide the outcome.
        let name = format!("HORUS_ENV_FLAG_SELFTEST_{}", std::process::id());
        assert_eq!(env_flag(&name), None, "an unset variable must not decide");

        // SAFETY: the name embeds this process's pid and is used by nothing
        // else, so no other thread can be reading it concurrently.
        unsafe { std::env::set_var(&name, "YES") };
        assert_eq!(env_flag(&name), Some(true));
        unsafe { std::env::set_var(&name, "off") };
        assert_eq!(env_flag(&name), Some(false));
        unsafe { std::env::set_var(&name, "banana") };
        assert_eq!(env_flag(&name), None);
        unsafe { std::env::remove_var(&name) };
    }
}
