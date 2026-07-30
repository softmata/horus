//! HMAC-SHA256 message authentication for safety-critical system topics.
//!
//! `horus_net` deliberately has **zero external dependencies** (see the crate
//! description and grok insight #197), so SHA-256 and HMAC are implemented here
//! rather than pulled from `sha2`/`hmac`. Both are small, well-specified, and
//! exercised against the published FIPS 180-4 / RFC 4231 vectors in the tests
//! at the bottom of this file.
//!
//! ## What this is for
//!
//! The networked e-stop channel (`_horus.estop`) was unauthenticated: any host
//! that could deliver a UDP datagram to the replicator port could halt a fleet
//! (GHSA-3frr-c2j9-hhr7). The remediation the authors had already planned — and
//! documented in `estop::handle_remote_estop` — is a MAC over the e-stop payload
//! keyed by material provisioned **off the wire**.
//!
//! This module provides that MAC. It deliberately does **not** reuse the
//! discovery `secret_hash`: that is a non-cryptographic FNV-1a value broadcast
//! in cleartext for peer *filtering*, and building authentication on it would be
//! fake security.
//!
//! ## Key provisioning
//!
//! `HORUS_ESTOP_KEY` holds the shared secret. It is never transmitted. The raw
//! string is hashed to a 32-byte key so operators can use a passphrase of any
//! length without a separate KDF step.
//!
//! When a key is configured, an e-stop without a valid tag is **rejected**. When
//! no key is configured, behaviour is unchanged (the loud one-time warning in
//! `estop` still fires) so existing deployments do not silently lose their
//! e-stop path on upgrade.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

/// Length of an HMAC-SHA256 tag, in bytes.
pub const TAG_LEN: usize = 32;

// ─── SHA-256 (FIPS 180-4) ───────────────────────────────────────────────────

const SHA256_BLOCK: usize = 64;

const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

/// Streaming SHA-256 state.
struct Sha256 {
    h: [u32; 8],
    buf: [u8; SHA256_BLOCK],
    buf_len: usize,
    total_len: u64,
}

impl Sha256 {
    fn new() -> Self {
        Self {
            h: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                0x5be0cd19,
            ],
            buf: [0u8; SHA256_BLOCK],
            buf_len: 0,
            total_len: 0,
        }
    }

    fn compress(&mut self, block: &[u8; SHA256_BLOCK]) {
        let mut w = [0u32; 64];
        for (i, word) in w.iter_mut().take(16).enumerate() {
            *word = u32::from_be_bytes([
                block[i * 4],
                block[i * 4 + 1],
                block[i * 4 + 2],
                block[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }

        let (mut a, mut b, mut c, mut d) = (self.h[0], self.h[1], self.h[2], self.h[3]);
        let (mut e, mut f, mut g, mut hh) = (self.h[4], self.h[5], self.h[6], self.h[7]);

        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);

            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        self.h[0] = self.h[0].wrapping_add(a);
        self.h[1] = self.h[1].wrapping_add(b);
        self.h[2] = self.h[2].wrapping_add(c);
        self.h[3] = self.h[3].wrapping_add(d);
        self.h[4] = self.h[4].wrapping_add(e);
        self.h[5] = self.h[5].wrapping_add(f);
        self.h[6] = self.h[6].wrapping_add(g);
        self.h[7] = self.h[7].wrapping_add(hh);
    }

    fn update(&mut self, mut data: &[u8]) {
        self.total_len = self.total_len.wrapping_add(data.len() as u64);

        if self.buf_len > 0 {
            let need = SHA256_BLOCK - self.buf_len;
            let take = need.min(data.len());
            self.buf[self.buf_len..self.buf_len + take].copy_from_slice(&data[..take]);
            self.buf_len += take;
            data = &data[take..];
            if self.buf_len == SHA256_BLOCK {
                let block = self.buf;
                self.compress(&block);
                self.buf_len = 0;
            }
        }

        while data.len() >= SHA256_BLOCK {
            let mut block = [0u8; SHA256_BLOCK];
            block.copy_from_slice(&data[..SHA256_BLOCK]);
            self.compress(&block);
            data = &data[SHA256_BLOCK..];
        }

        if !data.is_empty() {
            self.buf[..data.len()].copy_from_slice(data);
            self.buf_len = data.len();
        }
    }

    fn finalize(mut self) -> [u8; 32] {
        let bit_len = self.total_len.wrapping_mul(8);

        // Pad: 0x80, then zeros, then the 64-bit big-endian bit length.
        // `update` bumps total_len as it pads; that is harmless because bit_len
        // was captured first. The loop terminates because `update` compresses
        // and resets `buf_len` to 0 whenever the block fills, so `buf_len`
        // strictly cycles 0..64 and hits 56 exactly once per cycle.
        self.update(&[0x80]);
        while self.buf_len != SHA256_BLOCK - 8 {
            self.update(&[0x00]);
        }
        let len_bytes = bit_len.to_be_bytes();
        self.buf[SHA256_BLOCK - 8..].copy_from_slice(&len_bytes);
        let block = self.buf;
        self.compress(&block);

        let mut out = [0u8; 32];
        for (i, word) in self.h.iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
        }
        out
    }
}

/// One-shot SHA-256.
pub fn sha256(data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(data);
    h.finalize()
}

// ─── HMAC-SHA256 (RFC 2104) ─────────────────────────────────────────────────

/// HMAC-SHA256 over `msg` with `key`.
pub fn hmac_sha256(key: &[u8], msg: &[u8]) -> [u8; TAG_LEN] {
    // Keys longer than the block size are hashed down first (RFC 2104 §2).
    let mut k0 = [0u8; SHA256_BLOCK];
    if key.len() > SHA256_BLOCK {
        k0[..32].copy_from_slice(&sha256(key));
    } else {
        k0[..key.len()].copy_from_slice(key);
    }

    let mut ipad = [0x36u8; SHA256_BLOCK];
    let mut opad = [0x5cu8; SHA256_BLOCK];
    for i in 0..SHA256_BLOCK {
        ipad[i] ^= k0[i];
        opad[i] ^= k0[i];
    }

    let mut inner = Sha256::new();
    inner.update(&ipad);
    inner.update(msg);
    let inner_digest = inner.finalize();

    let mut outer = Sha256::new();
    outer.update(&opad);
    outer.update(&inner_digest);
    outer.finalize()
}

/// Constant-time equality. Runs in time proportional to `a.len()` only.
///
/// A short-circuiting `==` on a MAC tag leaks, byte by byte, how much of a
/// forged tag was correct — enough to forge a tag in 32×256 tries.
pub fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff: u8 = 0;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    // `black_box` keeps LLVM from turning the fold back into an early exit.
    std::hint::black_box(diff) == 0
}

// ─── Key provisioning ───────────────────────────────────────────────────────

/// Emitted at most once when a key is configured, so operators can confirm the
/// hardened path is actually active.
static KEY_ACTIVE_LOGGED: AtomicBool = AtomicBool::new(false);

/// The process-wide e-stop key, read once from `HORUS_ESTOP_KEY`.
///
/// `None` means no key is configured and the legacy unauthenticated behaviour
/// applies. An all-whitespace or empty value is treated as unset.
pub fn estop_key() -> Option<&'static [u8; 32]> {
    static KEY: OnceLock<Option<[u8; 32]>> = OnceLock::new();
    KEY.get_or_init(|| {
        let raw = std::env::var("HORUS_ESTOP_KEY").ok()?;
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return None;
        }
        // Hash the passphrase to a fixed-width key so any length works.
        Some(sha256(trimmed.as_bytes()))
    })
    .as_ref()
}

/// True if an e-stop key is configured, i.e. remote e-stop is authenticated.
pub fn estop_authenticated() -> bool {
    let active = estop_key().is_some();
    if active && !KEY_ACTIVE_LOGGED.swap(true, Ordering::Relaxed) {
        eprintln!(
            "[horus_net] Networked e-stop is AUTHENTICATED (HORUS_ESTOP_KEY is set). \
             Unauthenticated e-stop packets will be rejected."
        );
    }
    active
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{:02x}", b)).collect()
    }

    // ─── SHA-256, FIPS 180-4 published vectors ──────────────────────────────

    #[test]
    fn sha256_empty() {
        assert_eq!(
            hex(&sha256(b"")),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
    }

    #[test]
    fn sha256_abc() {
        assert_eq!(
            hex(&sha256(b"abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn sha256_two_block() {
        // The 56-byte vector — exercises the length-encoding edge where the
        // padding must spill into a second block.
        assert_eq!(
            hex(&sha256(
                b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
            )),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
    }

    #[test]
    fn sha256_million_a() {
        let data = vec![b'a'; 1_000_000];
        assert_eq!(
            hex(&sha256(&data)),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"
        );
    }

    #[test]
    fn sha256_exact_block_multiple() {
        // 64 bytes: buffer is exactly full when finalize() runs.
        let data = vec![b'x'; 64];
        assert_eq!(hex(&sha256(&data)).len(), 64);
        // Cross-check against the streaming path fed one byte at a time.
        let mut h = Sha256::new();
        for b in &data {
            h.update(&[*b]);
        }
        assert_eq!(h.finalize(), sha256(&data));
    }

    // ─── HMAC-SHA256, RFC 4231 test vectors ─────────────────────────────────

    #[test]
    fn hmac_rfc4231_case1() {
        let key = [0x0bu8; 20];
        let tag = hmac_sha256(&key, b"Hi There");
        assert_eq!(
            hex(&tag),
            "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
        );
    }

    #[test]
    fn hmac_rfc4231_case2() {
        let tag = hmac_sha256(b"Jefe", b"what do ya want for nothing?");
        assert_eq!(
            hex(&tag),
            "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
    }

    #[test]
    fn hmac_rfc4231_case3() {
        let key = [0xaau8; 20];
        let data = [0xddu8; 50];
        assert_eq!(
            hex(&hmac_sha256(&key, &data)),
            "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe"
        );
    }

    #[test]
    fn hmac_rfc4231_case6_long_key() {
        // 131-byte key — exercises the "hash the key down" branch.
        let key = [0xaau8; 131];
        let tag = hmac_sha256(&key, b"Test Using Larger Than Block-Size Key - Hash Key First");
        assert_eq!(
            hex(&tag),
            "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"
        );
    }

    // ─── Constant-time compare ──────────────────────────────────────────────

    #[test]
    fn ct_eq_matches_semantics_of_eq() {
        assert!(ct_eq(b"", b""));
        assert!(ct_eq(b"abcd", b"abcd"));
        assert!(!ct_eq(b"abcd", b"abce"));
        assert!(!ct_eq(b"abcd", b"abc"));
        // Differing in the FIRST byte must be just as "equal-looking" as the last.
        assert!(!ct_eq(b"Xbcd", b"abcd"));
    }

    #[test]
    fn tag_len_is_sha256_width() {
        assert_eq!(TAG_LEN, 32);
        assert_eq!(hmac_sha256(b"k", b"m").len(), TAG_LEN);
    }
}
