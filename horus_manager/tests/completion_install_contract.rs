//! What `install.sh` writes, and what `uninstall.sh` takes back.
//!
//! TOOL-1 was closed once already: `horus completion <shell>` was un-hidden,
//! it appears in `--help`, all five generators produce valid scripts, and
//! `install.sh` grew an `install_completions()`. `help_contract.rs` guards the
//! first two. Nothing guarded the third, and the third was broken:
//!
//!   * `SHELL_RC` was assigned inside the *else* branch of the PATH check, so
//!     every user who already had the install dir on PATH — everyone
//!     re-installing, and everyone with a Rust toolchain, which install.sh
//!     requires — reached `install_completions()` with `SHELL_RC` empty.
//!     `set -e` is on but `set -u` is not, so `[ -n "$SHELL_RC" ]` silently
//!     skipped the zsh `fpath` line, and the installer still printed
//!     "Shell completions installed (zsh)". A file zsh never loads, announced
//!     as a success.
//!   * Even on the branch that did append it, the `fpath` line went to the end
//!     of `.zshrc`, after the user's `compinit`. zsh reads `fpath` only at
//!     `compinit` time, so that is also a file zsh never loads.
//!   * bash hardcoded `~/.local/share/bash-completion/completions` while
//!     `install_man_page()` three lines below honoured `XDG_DATA_HOME`.
//!   * `uninstall.sh` still listed only the pre-move locations, so two of the
//!     three files the installer writes survived a full uninstall, along with
//!     the `.zshrc` edit.
//!
//! These tests run the real shell code — the section of `install.sh` from the
//! rc-file detection onwards, and the marked slices of `uninstall.sh` — against
//! a throw-away `$HOME` with a stub `horus` on PATH. A text search over
//! `install.sh` would have passed happily on every one of the bugs above.

#![cfg(unix)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

const INSTALL_ANCHOR: &str = "# --- Which rc file does this shell read? ---";

/// Everything in `install.sh` from the rc-file detection to the end of the
/// file: PATH wiring, shell integration, completions, man page and the closing
/// hint. Verbatim, so the test cannot drift away from the shipped script.
fn install_tail() -> String {
    let src = fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist");
    let at = src.find(INSTALL_ANCHOR).unwrap_or_else(|| {
        panic!("install.sh no longer contains the anchor {INSTALL_ANCHOR:?}; this test would otherwise silently cover nothing")
    });
    let tail = src[at..].to_string();
    // Self-check: if a refactor moves these out of the slice the test is no
    // longer testing what it claims to.
    for needed in [
        "install_completions()",
        "install_man_page()",
        "SHELL_RC=\"\"",
    ] {
        assert!(
            tail.contains(needed),
            "install.sh slice does not contain {needed:?} — the test would pass vacuously"
        );
    }
    tail
}

/// A marked region of `uninstall.sh`, inclusive of both marker lines.
fn uninstall_slice(begin: &str, end: &str) -> String {
    let src = fs::read_to_string(repo_root().join("uninstall.sh")).expect("uninstall.sh exists");
    let b = src
        .find(begin)
        .unwrap_or_else(|| panic!("uninstall.sh lost the marker {begin:?}"));
    let e = src
        .find(end)
        .unwrap_or_else(|| panic!("uninstall.sh lost the marker {end:?}"));
    assert!(b < e, "uninstall.sh markers {begin:?}/{end:?} are out of order");
    src[b..e + end.len()].to_string()
}

struct Sandbox {
    home: tempfile::TempDir,
}

impl Sandbox {
    fn new() -> Self {
        let home = tempfile::tempdir().expect("tempdir");
        let bin = home.path().join(".local/bin");
        fs::create_dir_all(&bin).unwrap();
        // A stub `horus`: this test is about *where* the installer puts the
        // generated files and whether the shell can find them, not about the
        // generator (help_contract.rs covers that).
        let stub = bin.join("horus");
        fs::write(
            &stub,
            "#!/bin/bash\n\
             case \"$1\" in\n\
             \x20 completion)\n\
             \x20   case \"$2\" in\n\
             \x20     zsh)  echo '#compdef horus'; echo '_horus() { :; }' ;;\n\
             \x20     bash) echo '_horus() { :; }' ;;\n\
             \x20     fish) echo 'complete -c horus' ;;\n\
             \x20     *) exit 1 ;;\n\
             \x20   esac ;;\n\
             \x20 man) echo '.TH horus 1' ;;\n\
             \x20 env) echo 'stub env' ;;\n\
             \x20 *) exit 1 ;;\n\
             esac\n",
        )
        .unwrap();
        let mut perms = fs::metadata(&stub).unwrap().permissions();
        std::os::unix::fs::PermissionsExt::set_mode(&mut perms, 0o755);
        fs::set_permissions(&stub, perms).unwrap();
        Sandbox { home }
    }

    fn path(&self) -> &Path {
        self.home.path()
    }

    fn read(&self, rel: &str) -> Option<String> {
        fs::read_to_string(self.path().join(rel)).ok()
    }

    fn exists(&self, rel: &str) -> bool {
        self.path().join(rel).exists()
    }

    fn run(&self, script: &str, shell: &str, path_already_configured: bool, xdg: &[(&str, &str)]) -> String {
        let script_path = self.path().join(".harness.sh");
        fs::write(&script_path, script).unwrap();

        // `PATH already configured` is the branch the refuter reproduced: the
        // install dir is already on PATH, which is the case for every upgrade.
        let run_path = if path_already_configured {
            format!("{}/.local/bin:/usr/bin:/bin", self.path().display())
        } else {
            "/usr/bin:/bin".to_string()
        };

        let mut cmd = Command::new("bash");
        cmd.arg(&script_path)
            .env_clear()
            .env("HOME", self.path())
            .env("SHELL", shell)
            .env("PATH", &run_path)
            // `horus env --init` edits shell rc files; that is TOOL-3's
            // territory and it is opted out of here so this test only observes
            // what install.sh itself writes.
            .env("HORUS_NO_SHELL_INTEGRATION", "1");
        for (k, v) in xdg {
            cmd.env(k, v);
        }
        let out = cmd.output().expect("bash must run");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(
            out.status.success(),
            "installer slice failed:\n{combined}"
        );
        combined
    }

    fn install(&self, shell: &str, path_already_configured: bool, xdg: &[(&str, &str)]) -> String {
        let script = format!(
            "INSTALL_DIR=\"$HOME/.local/bin\"\n\
             BINARY_NAME=horus\n\
             RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''\n\
             info(){{ echo \"  -> $1\"; }}\n\
             ok(){{ echo \"  ok $1\"; }}\n\
             warn(){{ echo \"  !  $1\"; }}\n\
             fail(){{ echo \"  x  $1\"; }}\n\
             INSTALL_START=$(date +%s)\n\
             VERSION=0.0.0-test\n\
             set -e\n\
             {}",
            install_tail()
        );
        self.run(&script, shell, path_already_configured, xdg)
    }

    fn uninstall(&self, xdg: &[(&str, &str)]) -> String {
        let script = format!(
            "HORUS_DIR=\"$HOME/.horus\"\n\
             GREEN=''; YELLOW=''; NC=''\n\
             REMOVED=0; SKIPPED=0\n\
             {paths}\n{removal}\n{profiles}\n\
             remove_completions_and_man_page\n\
             clean_shell_profiles\n",
            paths = uninstall_slice(
                "# >>> uninstall.sh: artifact paths >>>",
                "# <<< uninstall.sh: artifact paths <<<"
            ),
            removal = uninstall_slice(
                "# >>> uninstall.sh: completion + man removal >>>",
                "# <<< uninstall.sh: completion + man removal <<<"
            ),
            profiles = uninstall_slice(
                "# >>> uninstall.sh: shell profile cleanup >>>",
                "# <<< uninstall.sh: shell profile cleanup <<<"
            ),
        );
        // Same vacuity guard as the installer side.
        for needed in [
            "MAN_PAGE_PATHS=",
            "remove_completions_and_man_page()",
            "clean_shell_profiles()",
        ] {
            assert!(script.contains(needed), "uninstall.sh slice lost {needed:?}");
        }
        // SHELL is irrelevant to the uninstaller; it removes every location.
        self.run(&script, "/bin/bash", false, xdg)
    }
}

/// The refuter's reproduction, as a test.
///
/// zsh only scans `fpath`, and only at `compinit` time. A `_horus` that no
/// `fpath` line mentions is dead weight — which is TOOL-1's original defect,
/// "script generated, never reaches the shell", with a success message on top.
#[test]
fn zsh_completions_reach_the_shell_when_path_is_already_configured() {
    let sb = Sandbox::new();
    // A stock oh-my-zsh .zshrc: compinit runs inside oh-my-zsh.sh.
    fs::write(
        sb.path().join(".zshrc"),
        "source $ZSH/oh-my-zsh.sh\nalias ll=\"ls -l\"\n",
    )
    .unwrap();

    let out = sb.install("/bin/zsh", true, &[]);
    assert!(
        out.contains("PATH already configured"),
        "test did not exercise the branch it is about:\n{out}"
    );

    assert!(
        sb.exists(".zfunc/_horus"),
        "installer did not write the zsh completion script"
    );

    let zshrc = sb.read(".zshrc").expect(".zshrc");
    let fpath_line = zshrc
        .lines()
        .position(|l| l.contains("fpath=(") && l.contains(".zfunc"))
        .unwrap_or_else(|| {
            panic!(
                "~/.zfunc/_horus was written but .zshrc has no fpath line for it, \
                 so zsh will never load it:\n{zshrc}"
            )
        });
    let compinit_line = zshrc
        .lines()
        .position(|l| l.contains("compinit") || l.contains("oh-my-zsh.sh"))
        .expect("fixture .zshrc sources oh-my-zsh");
    assert!(
        fpath_line < compinit_line,
        "the fpath line is at {fpath_line} and compinit at {compinit_line}: zsh reads fpath \
         only at compinit time, so appending after it loads nothing:\n{zshrc}"
    );
}

/// The success line must not claim more than happened.
#[test]
fn the_installer_names_the_file_it_wrote() {
    let sb = Sandbox::new();
    fs::write(sb.path().join(".zshrc"), "autoload -Uz compinit\ncompinit\n").unwrap();
    let out = sb.install("/bin/zsh", true, &[]);
    let expected = sb.path().join(".zfunc/_horus");
    assert!(
        out.contains(&expected.display().to_string()),
        "the installer says completions are installed without saying where:\n{out}"
    );
}

/// A .zshrc with no completion system at all: an fpath line on its own would
/// still load nothing, so compinit has to be turned on too.
#[test]
fn a_zshrc_without_compinit_gets_one() {
    let sb = Sandbox::new();
    fs::write(sb.path().join(".zshrc"), "alias ll=\"ls -l\"\n").unwrap();
    sb.install("/bin/zsh", true, &[]);
    let zshrc = sb.read(".zshrc").expect(".zshrc");
    assert!(
        zshrc.contains("compinit"),
        "fpath was added to a .zshrc that never runs compinit, so nothing loads:\n{zshrc}"
    );
}

/// Re-running the installer is the normal upgrade path.
#[test]
fn reinstalling_does_not_duplicate_the_zshrc_block() {
    let sb = Sandbox::new();
    fs::write(sb.path().join(".zshrc"), "source $ZSH/oh-my-zsh.sh\n").unwrap();
    sb.install("/bin/zsh", true, &[]);
    sb.install("/bin/zsh", true, &[]);
    sb.install("/bin/zsh", true, &[]);
    let zshrc = sb.read(".zshrc").expect(".zshrc");
    assert_eq!(
        zshrc.matches("fpath=(").count(),
        1,
        "three installs left {} fpath lines:\n{zshrc}",
        zshrc.matches("fpath=(").count()
    );
}

/// `install_man_page()` honours XDG_DATA_HOME; the completion installer three
/// lines above it did not.
#[test]
fn bash_completions_honour_xdg_data_home() {
    let sb = Sandbox::new();
    let xdg = sb.path().join("xdg-data");
    let xdg_s = xdg.display().to_string();
    fs::write(sb.path().join(".bashrc"), "# bashrc\n").unwrap();

    sb.install("/bin/bash", true, &[("XDG_DATA_HOME", &xdg_s)]);

    assert!(
        xdg.join("bash-completion/completions/horus").exists(),
        "XDG_DATA_HOME is set but the completion went somewhere else; \
         bash-completion only scans $XDG_DATA_HOME/bash-completion/completions"
    );
    assert!(
        !sb.exists(".local/share/bash-completion/completions/horus"),
        "completion was written to the hardcoded ~/.local/share despite XDG_DATA_HOME"
    );
    // The man page is the sibling that always got this right; assert both agree.
    assert!(
        xdg.join("man/man1/horus.1").exists(),
        "man page ignored XDG_DATA_HOME"
    );
}

/// fish reads completions from $XDG_CONFIG_HOME/fish/completions.
#[test]
fn fish_completions_honour_xdg_config_home() {
    let sb = Sandbox::new();
    let xdg = sb.path().join("xdg-config");
    let xdg_s = xdg.display().to_string();

    sb.install("/usr/bin/fish", true, &[("XDG_CONFIG_HOME", &xdg_s)]);

    assert!(
        xdg.join("fish/completions/horus.fish").exists(),
        "fish completion ignored XDG_CONFIG_HOME"
    );
}

/// fish has no `export` builtin, so a POSIX export line in config.fish is a
/// syntax error and the whole file stops loading.
#[test]
fn the_fish_path_line_is_valid_fish() {
    let sb = Sandbox::new();
    sb.install("/usr/bin/fish", false, &[]);
    let cfg = sb
        .read(".config/fish/config.fish")
        .expect("installer must configure PATH for fish");
    assert!(
        !cfg.contains("export PATH="),
        "config.fish got a POSIX `export` line, which fish cannot parse:\n{cfg}"
    );
    assert!(
        cfg.contains("fish_add_path"),
        "config.fish has no fish-native PATH line:\n{cfg}"
    );
}

/// The upgrade path for a config.fish an earlier installer already poisoned.
///
/// The old guard was `grep -q "$INSTALL_DIR" "$SHELL_RC"`, which matches the
/// broken `export PATH="...:$PATH"` line just as happily as a working one — so
/// once that line was in place, every subsequent install saw "already
/// configured" and left fish unable to load its own config.
#[test]
fn a_config_fish_poisoned_by_an_older_installer_is_repaired() {
    let sb = Sandbox::new();
    let cfg_dir = sb.path().join(".config/fish");
    fs::create_dir_all(&cfg_dir).unwrap();
    let cfg = cfg_dir.join("config.fish");
    let poisoned = format!(
        "# my fish config\nexport PATH=\"{}/.local/bin:$PATH\"\nalias g=git\n",
        sb.path().display()
    );
    fs::write(&cfg, &poisoned).unwrap();

    sb.install("/usr/bin/fish", false, &[]);

    let after = fs::read_to_string(&cfg).unwrap();
    assert!(
        !after.contains("export PATH="),
        "the unparseable export line survived a re-install:\n{after}"
    );
    assert!(
        after.contains("fish_add_path"),
        "config.fish was left with no PATH line at all:\n{after}"
    );
    assert!(
        after.contains("alias g=git"),
        "the installer removed lines it did not write:\n{after}"
    );
}

/// Everything install.sh writes, uninstall.sh must take back. Two of the three
/// completion files and the man page used to survive, along with the .zshrc
/// edit — the uninstaller was still listing the locations install.sh had moved
/// away from.
#[test]
fn uninstall_removes_every_file_install_wrote_zsh() {
    let sb = Sandbox::new();
    fs::write(
        sb.path().join(".zshrc"),
        "source $ZSH/oh-my-zsh.sh\nalias ll=\"ls -l\"\n",
    )
    .unwrap();

    sb.install("/bin/zsh", true, &[]);
    assert!(sb.exists(".zfunc/_horus"));
    assert!(sb.exists(".local/share/man/man1/horus.1"));

    sb.uninstall(&[]);

    assert!(
        !sb.exists(".zfunc/_horus"),
        "~/.zfunc/_horus survived the uninstall"
    );
    assert!(
        !sb.exists(".local/share/man/man1/horus.1"),
        "the man page survived the uninstall: `man horus` still works after removing horus"
    );
    let zshrc = sb.read(".zshrc").expect(".zshrc");
    assert!(
        !zshrc.contains("fpath=("),
        "the fpath line install.sh appended survived the uninstall:\n{zshrc}"
    );
    assert!(
        zshrc.contains("source $ZSH/oh-my-zsh.sh") && zshrc.contains("alias ll"),
        "the uninstaller ate lines it did not write:\n{zshrc}"
    );
}

#[test]
fn uninstall_removes_every_file_install_wrote_bash_xdg() {
    let sb = Sandbox::new();
    let xdg = sb.path().join("xdg-data");
    let xdg_s = xdg.display().to_string();
    fs::write(sb.path().join(".bashrc"), "# bashrc\n").unwrap();

    sb.install("/bin/bash", true, &[("XDG_DATA_HOME", &xdg_s)]);
    assert!(xdg.join("bash-completion/completions/horus").exists());

    sb.uninstall(&[("XDG_DATA_HOME", &xdg_s)]);

    assert!(
        !xdg.join("bash-completion/completions/horus").exists(),
        "the XDG bash completion survived the uninstall"
    );
    assert!(
        !xdg.join("man/man1/horus.1").exists(),
        "the XDG man page survived the uninstall"
    );
}

#[test]
fn uninstall_removes_every_file_install_wrote_fish() {
    let sb = Sandbox::new();
    sb.install("/usr/bin/fish", true, &[]);
    assert!(sb.exists(".config/fish/completions/horus.fish"));

    sb.uninstall(&[]);

    assert!(
        !sb.exists(".config/fish/completions/horus.fish"),
        "the fish completion survived the uninstall"
    );
}

/// Every location install.sh can write must appear in uninstall.sh. The two
/// lists drifted apart silently once; this is the cheap check that catches the
/// next move even for a shell the behavioural tests above do not cover.
#[test]
fn the_uninstaller_lists_every_location_the_installer_writes() {
    let install = fs::read_to_string(repo_root().join("install.sh")).unwrap();
    let uninstall = fs::read_to_string(repo_root().join("uninstall.sh")).unwrap();

    for (fragment, what) in [
        ("/.zfunc", "zsh completion dir"),
        ("bash-completion/completions", "bash completion dir"),
        ("fish/completions", "fish completion dir"),
        ("man/man1", "man page dir"),
    ] {
        assert!(
            install.contains(fragment),
            "install.sh no longer writes {what} ({fragment}); update this test"
        );
        assert!(
            uninstall.contains(fragment),
            "install.sh writes {what} ({fragment}) but uninstall.sh never removes it"
        );
    }

    // The .zshrc block is delimited so the uninstaller can delete exactly it;
    // matching on "horus completion" never caught `fpath=(~/.zfunc $fpath)`.
    assert!(
        install.contains("# >>> horus completions >>>")
            && uninstall.contains("# >>> horus completions >>>"),
        "the marker install.sh writes into .zshrc and the one uninstall.sh deletes must match"
    );
}
