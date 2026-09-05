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
    assert!(
        b < e,
        "uninstall.sh markers {begin:?}/{end:?} are out of order"
    );
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

    fn run(
        &self,
        script: &str,
        shell: &str,
        path_already_configured: bool,
        xdg: &[(&str, &str)],
    ) -> String {
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
        assert!(out.status.success(), "installer slice failed:\n{combined}");
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
            assert!(
                script.contains(needed),
                "uninstall.sh slice lost {needed:?}"
            );
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
    fs::write(
        sb.path().join(".zshrc"),
        "autoload -Uz compinit\ncompinit\n",
    )
    .unwrap();
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

/// A prefix install must export `HORUS_PREFIX`, not just `PATH`.
///
/// Both readers take the prefix from the environment of the running `horus`
/// process and from nowhere else — `version.rs` says so outright: "HORUS_PREFIX
/// is the whole interface ... there is no second name to read, and no way to
/// discover a prefix install from outside its own tree". `run_rust`'s cache
/// roots and the version gate's state root are both derived from it.
///
/// The installer never put it in the user's environment: the rc file got a PATH
/// line only, `horus env --init` writes proxy functions and no exports, and the
/// prefix epilogue told the user to export `PATH` and `HORUS_SOURCE`. So for
/// every prefix user the cache root fell back to `~/.horus/cache` — deduped
/// away as the legacy root, meaning the prefix cache was never searched — and
/// the state root was `~/.horus`, which holds none of that install's files. The
/// Rust half of the change worked only for someone who guessed the variable.
#[test]
fn a_prefix_install_exports_horus_prefix_to_the_shell() {
    let sb = Sandbox::new();
    let prefix = sb.path().join("opt-horus");
    let script = format!(
        "INSTALL_DIR=\"{prefix}/bin\"\n\
         HORUS_PREFIX=\"{prefix}\"\n\
         BINARY_NAME=horus\n\
         RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''\n\
         info(){{ echo \"  -> $1\"; }}\n\
         ok(){{ echo \"  ok $1\"; }}\n\
         warn(){{ echo \"  !  $1\"; }}\n\
         fail(){{ echo \"  x  $1\"; }}\n\
         INSTALL_START=$(date +%s)\n\
         VERSION=0.0.0-test\n\
         set -e\n\
         {tail}",
        prefix = prefix.display(),
        tail = install_tail()
    );
    sb.run(&script, "/bin/bash", false, &[]);

    let rc = sb
        .read(".bashrc")
        .expect("the installer must write to the rc file");
    assert!(
        rc.contains("export HORUS_PREFIX="),
        "a prefix install must export HORUS_PREFIX — without it `horus run` \
         never searches the prefix cache and the version gate reads a state \
         root that holds none of this install's files. rc was:\n{rc}"
    );
    assert!(
        rc.contains(&prefix.display().to_string()),
        "the exported value must be the prefix that was installed to:\n{rc}"
    );
}

/// ...including on a re-install, where PATH is already configured.
///
/// This is the population the line exists for: a prefix install puts
/// `${HORUS_PREFIX}/bin` on PATH, so every upgrade and re-install takes the
/// "PATH already configured" arm. The export first landed inside the `else` arm
/// of that check, which made it dead for exactly those users — the same mistake
/// install.sh already records 60 lines above for the shell-integration block
/// that used to live there.
#[test]
fn a_prefix_install_exports_horus_prefix_on_a_reinstall_too() {
    let sb = Sandbox::new();
    let prefix = sb.path().join("opt-horus");
    let script = format!(
        "INSTALL_DIR=\"{prefix}/bin\"\n\
         HORUS_PREFIX=\"{prefix}\"\n\
         BINARY_NAME=horus\n\
         RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''\n\
         info(){{ echo \"  -> $1\"; }}\n\
         ok(){{ echo \"  ok $1\"; }}\n\
         warn(){{ echo \"  !  $1\"; }}\n\
         fail(){{ echo \"  x  $1\"; }}\n\
         INSTALL_START=$(date +%s)\n\
         VERSION=0.0.0-test\n\
         PATH=\"$INSTALL_DIR:$PATH\"\n\
         set -e\n\
         {tail}",
        prefix = prefix.display(),
        tail = install_tail()
    );
    // INSTALL_DIR is on PATH above, which is what every re-install of a prefix
    // install looks like.
    let out = sb.run(&script, "/bin/bash", false, &[]);
    assert!(
        out.contains("PATH already configured"),
        "precondition: this test must exercise the already-configured arm:\n{out}"
    );

    let rc = sb.read(".bashrc").unwrap_or_default();
    assert!(
        rc.contains("export HORUS_PREFIX="),
        "a re-install must still export HORUS_PREFIX — it is the population \
         that most needs it, since a prefix install always puts its bin on \
         PATH. rc was:\n{rc}"
    );
}

/// A stale prefix line is replaced, not counted as already configured.
#[test]
fn a_changed_prefix_replaces_the_old_export() {
    let sb = Sandbox::new();
    std::fs::write(
        sb.path().join(".bashrc"),
        "export HORUS_PREFIX=\"/old/location\"\n",
    )
    .unwrap();

    let prefix = sb.path().join("opt-horus");
    let script = format!(
        "INSTALL_DIR=\"{prefix}/bin\"\n\
         HORUS_PREFIX=\"{prefix}\"\n\
         BINARY_NAME=horus\n\
         RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''\n\
         info(){{ echo \"  -> $1\"; }}\n\
         ok(){{ echo \"  ok $1\"; }}\n\
         warn(){{ echo \"  !  $1\"; }}\n\
         fail(){{ echo \"  x  $1\"; }}\n\
         INSTALL_START=$(date +%s)\n\
         VERSION=0.0.0-test\n\
         PATH=\"$INSTALL_DIR:$PATH\"\n\
         set -e\n\
         {tail}",
        prefix = prefix.display(),
        tail = install_tail()
    );
    sb.run(&script, "/bin/bash", false, &[]);

    let rc = sb.read(".bashrc").unwrap_or_default();
    assert!(
        rc.contains(&prefix.display().to_string()),
        "the new prefix must be exported:\n{rc}"
    );
    assert!(
        !rc.contains("/old/location"),
        "a stale prefix line must be removed, not left to win by being first \
         in the file:\n{rc}"
    );
}

/// The `export HORUS_PREFIX` line must be fish-safe.
///
/// fish has no `export` builtin, and a POSIX export line stops `config.fish`
/// loading at that point — the exact failure this file already records for the
/// PATH line, where a poisoned config stayed poisoned through every upgrade
/// because the guard found the broken line and skipped writing a working one.
#[test]
fn the_prefix_export_uses_fish_syntax_in_config_fish() {
    let sb = Sandbox::new();
    let prefix = sb.path().join("opt-horus");
    let script = format!(
        "INSTALL_DIR=\"{prefix}/bin\"\n\
         HORUS_PREFIX=\"{prefix}\"\n\
         BINARY_NAME=horus\n\
         RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''\n\
         info(){{ echo \"  -> $1\"; }}\n\
         ok(){{ echo \"  ok $1\"; }}\n\
         warn(){{ echo \"  !  $1\"; }}\n\
         fail(){{ echo \"  x  $1\"; }}\n\
         INSTALL_START=$(date +%s)\n\
         VERSION=0.0.0-test\n\
         set -e\n\
         {tail}",
        prefix = prefix.display(),
        tail = install_tail()
    );
    sb.run(&script, "/usr/bin/fish", false, &[]);

    let rc = sb
        .read(".config/fish/config.fish")
        .expect("the installer must write config.fish for a fish shell");
    assert!(
        rc.contains("set -gx HORUS_PREFIX"),
        "config.fish needs `set -gx`, not `export`:\n{rc}"
    );
    assert!(
        !rc.contains("export HORUS_PREFIX"),
        "a POSIX export line stops config.fish loading at that point:\n{rc}"
    );
}

/// The `check_rust_version` gate, verbatim from `install.sh`.
fn rust_gate_slice() -> String {
    let src = fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist");
    let begin = "check_rust_version() {";
    let end = "    ok \"Rust $found (>= $required required)\"\n}";
    let b = src
        .find(begin)
        .unwrap_or_else(|| panic!("install.sh lost the marker {begin:?}"));
    let e = src
        .find(end)
        .unwrap_or_else(|| panic!("install.sh lost the marker {end:?}"));
    assert!(b < e, "install.sh markers are out of order");
    let slice = src[b..e + end.len()].to_string();
    // Without these the harness would be exercising an empty shell function and
    // every assertion below would hold vacuously.
    for needed in [
        "rust-version",
        "rustc --version",
        "$HORUS_SRC_DIR/Cargo.toml",
    ] {
        assert!(
            slice.contains(needed),
            "the check_rust_version slice no longer contains {needed:?} — this test would pass vacuously"
        );
    }
    slice
}

/// Run the gate against a stub `rustc` reporting `rustc_version`, with the
/// workspace floor set to `required`. `sort_has_dash_v` chooses whether the
/// `sort` on PATH understands `-V`. Returns the gate's output plus a
/// PROCEEDED/ABORTED verdict.
fn run_rust_gate(rustc_version: &str, required: &str, sort_has_dash_v: bool) -> String {
    let sb = Sandbox::new();
    let stub = sb.path().join("stub");
    fs::create_dir_all(&stub).unwrap();
    fs::create_dir_all(sb.path().join("src")).unwrap();
    fs::write(
        sb.path().join("src/Cargo.toml"),
        format!("[workspace.package]\nrust-version = \"{required}\"\n"),
    )
    .unwrap();

    let rustc = stub.join("rustc");
    fs::write(
        &rustc,
        format!("#!/bin/bash\necho 'rustc {rustc_version} (deadbeef 2026-01-01)'\n"),
    )
    .unwrap();
    // A `sort` that rejects -V, the way any sort predating the GNU/BSD version
    // extension does. The point is not one named platform: it is that the gate
    // must not convict a toolchain on a comparison it was unable to perform.
    if !sort_has_dash_v {
        let sort = stub.join("sort");
        fs::write(
            &sort,
            "#!/bin/bash\nfor a in \"$@\"; do case \"$a\" in -V|--version-sort)\n\
             echo 'sort: illegal option -- V' >&2; exit 2 ;; esac; done\nexec /usr/bin/sort \"$@\"\n",
        )
        .unwrap();
        set_exec(&sort);
    }
    set_exec(&rustc);

    let script = format!(
        "set -u\n\
         PATH=\"$HOME/stub:$PATH\"\n\
         HORUS_SRC_DIR=\"$HOME/src\"\n\
         fail() {{ echo \"GATE_FAIL: $*\"; }}\n\
         ok()   {{ echo \"GATE_OK: $*\"; }}\n\
         {}\n\
         if ( check_rust_version ); then echo PROCEEDED; else echo ABORTED; fi\n",
        rust_gate_slice()
    );
    sb.run(&script, "/bin/bash", true, &[])
}

fn set_exec(p: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(p, fs::Permissions::from_mode(0o755)).unwrap();
    }
}

#[test]
fn the_rust_version_gate_does_not_abort_when_it_cannot_compare() {
    let out = run_rust_gate("1.90.0", "1.85", false);
    assert!(
        out.contains("PROCEEDED"),
        "a `sort` without -V made the version gate abort an install on a toolchain \
         that satisfies the floor (1.90.0 >= 1.85). A gate that cannot perform its \
         comparison must fail open, as the two guards above it already do:\n{out}"
    );
    assert!(
        !out.contains("or newer is required"),
        "the installer accused a perfectly good toolchain of being too old:\n{out}"
    );
}

#[test]
fn the_rust_version_gate_still_rejects_a_genuinely_old_toolchain() {
    let out = run_rust_gate("1.70.0", "1.85", true);
    assert!(
        out.contains("ABORTED") && out.contains("or newer is required"),
        "failing open must not mean failing blind: 1.70.0 is below the 1.85 floor \
         and the gate has to say so:\n{out}"
    );
}

#[test]
fn the_rust_version_gate_compares_numerically_not_lexically() {
    let out = run_rust_gate("1.100.0", "1.9", true);
    assert!(
        out.contains("PROCEEDED"),
        "1.100 is newer than 1.9; a string compare says otherwise and would reject \
         every toolchain once the minor version passes 9:\n{out}"
    );
}
