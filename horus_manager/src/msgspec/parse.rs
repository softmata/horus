//! `.hmsg` — the message definition format.
//!
//! ```text
//! /// Weather station reading.
//! #[topic = "weather.data"]
//! WeatherData {
//!     /// Celsius
//!     temperature: f32,
//!     humidity: f32,
//!     wind: Vector3,
//!     history: [f32; 16],
//!     timestamp_ns: u64,
//! }
//! ```
//!
//! The item body is token-for-token what `message!` accepts, so a definition
//! can be moved between the two without editing.
//!
//! Everything this parser accepts must be POD: every artifact it feeds is
//! `#[repr(C)]`, and the C ABI reads and writes messages with a raw
//! `ptr::read`/`ptr::write`. A `String` or a `Vec<T>` there is not a
//! limitation, it is memory corruption — `horus_cpp` already excludes five
//! upstream types for exactly this reason. So `String`, `Vec`, references,
//! generics and enums are rejected by name, with a diagnostic that says why
//! rather than "unexpected token".

use std::path::{Path, PathBuf};

use super::{Diag, Field, MsgDef, Prim, Resolved, Type};

// ─── Tokenizer ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Ident(String),
    Int(usize),
    Str(String),
    /// `///` doc line, contents trimmed.
    Doc(String),
    Punct(char),
}

#[derive(Debug, Clone)]
struct Spanned {
    tok: Tok,
    line: usize,
    col: usize,
}

struct Lexer<'a> {
    src: &'a [u8],
    pos: usize,
    line: usize,
    col: usize,
    file: PathBuf,
}

impl<'a> Lexer<'a> {
    fn new(src: &'a str, file: &Path) -> Self {
        Self {
            src: src.as_bytes(),
            pos: 0,
            line: 1,
            col: 1,
            file: file.to_path_buf(),
        }
    }

    fn bump(&mut self) -> Option<u8> {
        let b = *self.src.get(self.pos)?;
        self.pos += 1;
        if b == b'\n' {
            self.line += 1;
            self.col = 1;
        } else {
            self.col += 1;
        }
        Some(b)
    }

    fn peek(&self) -> Option<u8> {
        self.src.get(self.pos).copied()
    }

    fn peek_at(&self, n: usize) -> Option<u8> {
        self.src.get(self.pos + n).copied()
    }

    fn tokenize(mut self) -> Result<Vec<Spanned>, Vec<Diag>> {
        let mut out = Vec::new();
        let mut errors = Vec::new();

        loop {
            // Whitespace.
            while matches!(self.peek(), Some(b) if b.is_ascii_whitespace()) {
                self.bump();
            }
            let Some(b) = self.peek() else { break };
            let (line, col) = (self.line, self.col);

            // `#` starts a comment, except `#[` which is an attribute.
            if b == b'#' && self.peek_at(1) != Some(b'[') {
                while let Some(c) = self.peek() {
                    if c == b'\n' {
                        break;
                    }
                    self.bump();
                }
                continue;
            }

            // `///` doc, `//` comment.
            if b == b'/' && self.peek_at(1) == Some(b'/') {
                let is_doc = self.peek_at(2) == Some(b'/');
                self.bump();
                self.bump();
                if is_doc {
                    self.bump();
                }
                let start = self.pos;
                while let Some(c) = self.peek() {
                    if c == b'\n' {
                        break;
                    }
                    self.bump();
                }
                if is_doc {
                    let text = String::from_utf8_lossy(&self.src[start..self.pos])
                        .trim()
                        .to_string();
                    out.push(Spanned {
                        tok: Tok::Doc(text),
                        line,
                        col,
                    });
                }
                continue;
            }

            if b.is_ascii_alphabetic() || b == b'_' {
                let start = self.pos;
                while matches!(self.peek(), Some(c) if c.is_ascii_alphanumeric() || c == b'_') {
                    self.bump();
                }
                let s = String::from_utf8_lossy(&self.src[start..self.pos]).into_owned();
                out.push(Spanned {
                    tok: Tok::Ident(s),
                    line,
                    col,
                });
                continue;
            }

            if b.is_ascii_digit() {
                let start = self.pos;
                while matches!(self.peek(), Some(c) if c.is_ascii_digit() || c == b'_') {
                    self.bump();
                }
                let raw = String::from_utf8_lossy(&self.src[start..self.pos]).replace('_', "");
                match raw.parse::<usize>() {
                    Ok(n) => out.push(Spanned {
                        tok: Tok::Int(n),
                        line,
                        col,
                    }),
                    Err(_) => errors.push(Diag::new(
                        &self.file,
                        line,
                        col,
                        format!("`{raw}` is not a usable array length"),
                    )),
                }
                continue;
            }

            if b == b'"' {
                self.bump();
                let start = self.pos;
                let mut terminated = false;
                while let Some(c) = self.peek() {
                    if c == b'"' {
                        terminated = true;
                        break;
                    }
                    if c == b'\n' {
                        break;
                    }
                    self.bump();
                }
                let s = String::from_utf8_lossy(&self.src[start..self.pos]).into_owned();
                if terminated {
                    self.bump();
                    out.push(Spanned {
                        tok: Tok::Str(s),
                        line,
                        col,
                    });
                } else {
                    errors.push(Diag::new(&self.file, line, col, "unterminated string"));
                }
                continue;
            }

            if matches!(
                b,
                b'{' | b'}'
                    | b'['
                    | b']'
                    | b':'
                    | b','
                    | b';'
                    | b'='
                    | b'#'
                    | b'<'
                    | b'>'
                    | b'&'
                    | b'('
                    | b')'
            ) {
                self.bump();
                out.push(Spanned {
                    tok: Tok::Punct(b as char),
                    line,
                    col,
                });
                continue;
            }

            self.bump();
            errors.push(Diag::new(
                &self.file,
                line,
                col,
                format!("unexpected character `{}`", b as char),
            ));
        }

        if errors.is_empty() {
            Ok(out)
        } else {
            Err(errors)
        }
    }
}

// ─── Parser ─────────────────────────────────────────────────────────────────

struct Parser {
    toks: Vec<Spanned>,
    pos: usize,
    file: PathBuf,
    errors: Vec<Diag>,
}

/// Types that are valid Rust and cannot cross the ABI, named so the diagnostic
/// can say why rather than "expected a type".
const NOT_POD: &[(&str, &str)] = &[
    ("String", "a String owns a heap allocation, which cannot be shared between processes. Use a fixed array like `[u8; 64]`."),
    ("Vec", "a Vec owns a heap allocation, which cannot be shared between processes. Use a fixed array like `[f32; 16]`."),
    ("Box", "a Box is a pointer into this process's heap, which means nothing in another process."),
    ("Option", "Option has no guaranteed layout across the ABI. Use a sentinel value, or a separate `valid: bool` field."),
    ("HashMap", "a HashMap owns heap allocations, which cannot be shared between processes."),
    ("str", "a string slice is a pointer and a length. Use a fixed array like `[u8; 64]`."),
];

impl Parser {
    fn peek(&self) -> Option<&Spanned> {
        self.toks.get(self.pos)
    }

    fn next(&mut self) -> Option<Spanned> {
        let t = self.toks.get(self.pos).cloned();
        if t.is_some() {
            self.pos += 1;
        }
        t
    }

    fn at_punct(&self, c: char) -> bool {
        matches!(self.peek(), Some(Spanned { tok: Tok::Punct(p), .. }) if *p == c)
    }

    fn eat_punct(&mut self, c: char) -> bool {
        if self.at_punct(c) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn here(&self) -> (usize, usize) {
        self.peek()
            .map(|s| (s.line, s.col))
            .or_else(|| self.toks.last().map(|s| (s.line, s.col)))
            .unwrap_or((1, 1))
    }

    fn error(&mut self, message: impl Into<String>) {
        let (line, col) = self.here();
        self.errors.push(Diag::new(&self.file, line, col, message));
    }

    fn error_with_help(&mut self, message: impl Into<String>, help: impl Into<String>) {
        let (line, col) = self.here();
        self.errors
            .push(Diag::new(&self.file, line, col, message).with_help(help));
    }

    /// Skip to just past the next `}` at depth zero, so one bad message does
    /// not cascade into every message after it.
    fn recover(&mut self) {
        let mut depth = 0usize;
        while let Some(s) = self.next() {
            match s.tok {
                Tok::Punct('{') => depth += 1,
                Tok::Punct('}') => {
                    if depth <= 1 {
                        return;
                    }
                    depth -= 1;
                }
                _ => {}
            }
        }
    }

    fn parse_docs(&mut self) -> Vec<String> {
        let mut docs = Vec::new();
        while let Some(Spanned {
            tok: Tok::Doc(d), ..
        }) = self.peek()
        {
            docs.push(d.clone());
            self.pos += 1;
        }
        docs
    }

    /// `#[topic = "..."]`. Unknown attributes are an error, not a shrug: a
    /// misspelled attribute that is silently ignored produces an artifact that
    /// is missing something the author asked for.
    fn parse_attrs(&mut self) -> Option<String> {
        let mut topic = None;
        while self.at_punct('#') {
            self.pos += 1;
            if !self.eat_punct('[') {
                self.error("expected `[` after `#`");
                return topic;
            }
            let name = match self.next() {
                Some(Spanned {
                    tok: Tok::Ident(n), ..
                }) => n,
                _ => {
                    self.error("expected an attribute name after `#[`");
                    return topic;
                }
            };
            match name.as_str() {
                "topic" => {
                    if !self.eat_punct('=') {
                        self.error("`#[topic]` needs a value: `#[topic = \"name\"]`");
                    } else {
                        match self.next() {
                            Some(Spanned {
                                tok: Tok::Str(s), ..
                            }) => topic = Some(s),
                            _ => self.error("`#[topic]` takes a string: `#[topic = \"name\"]`"),
                        }
                    }
                }
                other => {
                    self.error_with_help(
                        format!("unknown attribute `{other}`"),
                        "the only attribute is `#[topic = \"name\"]`",
                    );
                    // Consume to the closing bracket so recovery is sane.
                    while !self.at_punct(']') && self.peek().is_some() {
                        self.pos += 1;
                    }
                }
            }
            if !self.eat_punct(']') {
                self.error("expected `]` to close the attribute");
                return topic;
            }
        }
        topic
    }

    fn parse_type(&mut self) -> Option<Type> {
        if self.eat_punct('[') {
            let elem = self.parse_type()?;
            if !self.eat_punct(';') {
                self.error_with_help(
                    "expected `;` in array type",
                    "arrays are written `[T; N]`, with a length known at compile time",
                );
                return None;
            }
            let len = match self.next() {
                Some(Spanned {
                    tok: Tok::Int(n), ..
                }) => n,
                _ => {
                    self.error_with_help(
                        "expected an array length",
                        "arrays are written `[T; N]`; a length that is not a literal number \
                         cannot be shared between processes",
                    );
                    return None;
                }
            };
            if len == 0 {
                self.error("array length must be at least 1");
                return None;
            }
            if !self.eat_punct(']') {
                self.error("expected `]` to close the array type");
                return None;
            }
            return Some(Type::Array(Box::new(elem), len));
        }

        if self.at_punct('&') {
            self.error_with_help(
                "references cannot appear in a message",
                "a reference is a pointer into one process's memory. Store the value itself.",
            );
            return None;
        }

        let (line, col) = self.here();
        let name = match self.next() {
            Some(Spanned {
                tok: Tok::Ident(n), ..
            }) => n,
            _ => {
                self.error("expected a type");
                return None;
            }
        };

        if let Some((_, why)) = NOT_POD.iter().find(|(n, _)| *n == name) {
            self.errors.push(
                Diag::new(
                    &self.file,
                    line,
                    col,
                    format!("`{name}` cannot be used in a message"),
                )
                .with_help(*why),
            );
            // Swallow any generic arguments so recovery lands cleanly.
            if self.at_punct('<') {
                let mut depth = 0usize;
                while let Some(s) = self.peek() {
                    match s.tok {
                        Tok::Punct('<') => depth += 1,
                        Tok::Punct('>') => {
                            depth -= 1;
                            self.pos += 1;
                            if depth == 0 {
                                break;
                            }
                            continue;
                        }
                        _ => {}
                    }
                    self.pos += 1;
                }
            }
            return None;
        }

        if self.at_punct('<') {
            self.errors.push(
                Diag::new(
                    &self.file,
                    line,
                    col,
                    format!("`{name}` takes type parameters, which a message cannot"),
                )
                .with_help("every field must have one fixed layout; a generic does not"),
            );
            return None;
        }

        if let Some(p) = Prim::from_name(&name) {
            return Some(Type::Prim(p));
        }

        // Unresolved for now; `resolve` fills in the paths once every message
        // in the package is known.
        Some(Type::Ref(Resolved {
            short: name,
            rust_path: String::new(),
            cpp_path: String::new(),
            external: false,
        }))
    }

    fn parse_message(&mut self) -> Option<MsgDef> {
        let doc = self.parse_docs();
        let topic = self.parse_attrs();
        // An attribute may be followed by more docs.
        let mut doc = doc;
        doc.extend(self.parse_docs());

        let (line, _) = self.here();
        let name = match self.next() {
            Some(Spanned {
                tok: Tok::Ident(n), ..
            }) => n,
            Some(Spanned { tok, line, col }) => {
                self.errors.push(Diag::new(
                    &self.file,
                    line,
                    col,
                    format!("expected a message name, found {}", describe(&tok)),
                ));
                self.recover();
                return None;
            }
            None => return None,
        };

        if !name.chars().next().is_some_and(|c| c.is_ascii_uppercase()) {
            self.errors.push(
                Diag::new(
                    &self.file,
                    line,
                    1,
                    format!("message name `{name}` must be CamelCase"),
                )
                .with_help("the name becomes a Rust struct, a C++ struct and a Python class"),
            );
        }

        if !self.eat_punct('{') {
            self.error_with_help(
                format!("expected `{{` after message name `{name}`"),
                "a message is written `Name { field: type, ... }`",
            );
            self.recover();
            return None;
        }

        let mut fields: Vec<Field> = Vec::new();
        loop {
            if self.eat_punct('}') {
                break;
            }
            if self.peek().is_none() {
                self.error(format!("unterminated message `{name}`"));
                return None;
            }

            let fdoc = self.parse_docs();
            if self.eat_punct('}') {
                break;
            }

            let (fline, fcol) = self.here();
            let fname = match self.next() {
                Some(Spanned {
                    tok: Tok::Ident(n), ..
                }) => n,
                Some(Spanned { tok, line, col }) => {
                    self.errors.push(Diag::new(
                        &self.file,
                        line,
                        col,
                        format!("expected a field name, found {}", describe(&tok)),
                    ));
                    self.recover();
                    return None;
                }
                None => return None,
            };

            if fname == "pub" {
                self.errors.push(
                    Diag::new(&self.file, fline, fcol, "fields are public; drop `pub`")
                        .with_help("every field of a message is part of its wire layout"),
                );
            }

            if !self.eat_punct(':') {
                self.error_with_help(
                    format!("expected `:` after field `{fname}`"),
                    "fields are written `name: type`",
                );
                self.recover();
                return None;
            }

            let Some(ty) = self.parse_type() else {
                self.recover();
                return None;
            };

            if let Some(prev) = fields.iter().find(|f| f.name == fname) {
                self.errors.push(
                    Diag::new(
                        &self.file,
                        fline,
                        fcol,
                        format!("duplicate field `{fname}`"),
                    )
                    .with_help(format!("first declared on line {}", prev.line)),
                );
            }

            fields.push(Field {
                name: fname,
                ty,
                doc: fdoc,
                line: fline,
                col: fcol,
            });

            if self.eat_punct(',') {
                continue;
            }
            if self.at_punct(';') {
                self.error_with_help("fields are separated by `,`, not `;`", "use `,`");
                self.pos += 1;
                continue;
            }
            if self.eat_punct('}') {
                break;
            }
            self.error("expected `,` or `}` after a field");
            self.recover();
            return None;
        }

        if fields.is_empty() {
            self.errors.push(
                Diag::new(
                    &self.file,
                    line,
                    1,
                    format!("message `{name}` has no fields"),
                )
                .with_help("a zero-sized message carries nothing"),
            );
            return None;
        }

        Some(MsgDef {
            name,
            topic,
            doc,
            fields,
            src: self.file.clone(),
            line,
        })
    }
}

fn describe(tok: &Tok) -> String {
    match tok {
        Tok::Ident(s) => format!("`{s}`"),
        Tok::Int(n) => format!("`{n}`"),
        Tok::Str(s) => format!("string `{s}`"),
        Tok::Doc(_) => "a doc comment".to_string(),
        Tok::Punct(c) => format!("`{c}`"),
    }
}

/// Parse one `.hmsg` file. Reports every error it can, not just the first.
pub fn parse_file(src: &str, path: &Path) -> Result<Vec<MsgDef>, Vec<Diag>> {
    let toks = Lexer::new(src, path).tokenize()?;
    let mut p = Parser {
        toks,
        pos: 0,
        file: path.to_path_buf(),
        errors: Vec::new(),
    };

    let mut out = Vec::new();
    while p.peek().is_some() {
        if let Some(m) = p.parse_message() {
            out.push(m);
        }
    }

    if p.errors.is_empty() {
        Ok(out)
    } else {
        Err(p.errors)
    }
}
