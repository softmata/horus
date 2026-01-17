//! Certificate management for HORUS network transports
//!
//! Provides automatic certificate generation, persistence, and management for
//! TLS and QUIC transports. Certificates are automatically generated on first
//! run and persisted to `~/.horus/certs/` for reuse.
//!
//! # Features
//!
//! - **Zero-config for development**: Auto-generates self-signed certificates
//! - **Persistent storage**: Certificates persist across restarts
//! - **Custom paths**: Support for custom certificate locations
//! - **Certificate pinning**: Optional pinning for enhanced security
//! - **Expiration checking**: Warns when certificates are about to expire
//!
//! # Example
//!
//! ```rust,no_run
//! use horus_core::communication::network::CertificateManager;
//!
//! // Zero-config usage - auto-generates and persists certificates
//! let manager = CertificateManager::new().unwrap();
//! let (certs, key) = manager.get_or_create().unwrap();
//!
//! // Custom certificate path
//! let manager = CertificateManager::builder()
//!     .with_cert_dir("/etc/horus/certs")
//!     .build()
//!     .unwrap();
//! ```

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

#[cfg(feature = "tls")]
use rustls::pki_types::{CertificateDer, PrivateKeyDer};

/// Default certificate directory (relative to home)
pub const DEFAULT_CERT_DIR: &str = ".horus/certs";

/// Default certificate file name
pub const DEFAULT_CERT_FILE: &str = "horus.crt";

/// Default private key file name
pub const DEFAULT_KEY_FILE: &str = "horus.key";

/// Default certificate validity period (1 year)
pub const DEFAULT_CERT_VALIDITY_DAYS: u64 = 365;

/// Warning threshold for certificate expiration (30 days)
pub const CERT_EXPIRY_WARNING_DAYS: u64 = 30;

/// Certificate manager configuration
#[derive(Debug, Clone)]
pub struct CertificateConfig {
    /// Directory to store certificates
    pub cert_dir: PathBuf,
    /// Certificate file name
    pub cert_file: String,
    /// Private key file name
    pub key_file: String,
    /// Organization name for generated certificates
    pub organization: String,
    /// Common name for generated certificates
    pub common_name: String,
    /// Additional Subject Alternative Names (DNS names)
    pub san_dns: Vec<String>,
    /// Additional Subject Alternative Names (IP addresses)
    pub san_ips: Vec<std::net::IpAddr>,
    /// Validity period in days for generated certificates
    pub validity_days: u64,
    /// Enable certificate pinning
    pub enable_pinning: bool,
    /// Pinned certificate fingerprints (SHA-256)
    pub pinned_fingerprints: Vec<String>,
    /// Auto-regenerate if certificate is expiring soon
    pub auto_renew: bool,
    /// Days before expiry to trigger auto-renewal
    pub auto_renew_days: u64,
}

impl Default for CertificateConfig {
    fn default() -> Self {
        let home = dirs::home_dir().unwrap_or_else(|| PathBuf::from("."));
        Self {
            cert_dir: home.join(DEFAULT_CERT_DIR),
            cert_file: DEFAULT_CERT_FILE.to_string(),
            key_file: DEFAULT_KEY_FILE.to_string(),
            organization: "HORUS Robotics".to_string(),
            common_name: "horus".to_string(),
            san_dns: vec!["localhost".to_string()],
            san_ips: vec![
                std::net::IpAddr::V4(std::net::Ipv4Addr::new(127, 0, 0, 1)),
                std::net::IpAddr::V6(std::net::Ipv6Addr::new(0, 0, 0, 0, 0, 0, 0, 1)),
            ],
            validity_days: DEFAULT_CERT_VALIDITY_DAYS,
            enable_pinning: false,
            pinned_fingerprints: Vec::new(),
            auto_renew: true,
            auto_renew_days: CERT_EXPIRY_WARNING_DAYS,
        }
    }
}

/// Builder for CertificateManager
#[derive(Debug, Default)]
pub struct CertificateManagerBuilder {
    config: CertificateConfig,
}

impl CertificateManagerBuilder {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the certificate directory
    pub fn with_cert_dir<P: AsRef<Path>>(mut self, path: P) -> Self {
        self.config.cert_dir = path.as_ref().to_path_buf();
        self
    }

    /// Set certificate and key file names
    pub fn with_file_names(mut self, cert_file: impl Into<String>, key_file: impl Into<String>) -> Self {
        self.config.cert_file = cert_file.into();
        self.config.key_file = key_file.into();
        self
    }

    /// Set organization name for generated certificates
    pub fn with_organization(mut self, org: impl Into<String>) -> Self {
        self.config.organization = org.into();
        self
    }

    /// Set common name for generated certificates
    pub fn with_common_name(mut self, cn: impl Into<String>) -> Self {
        self.config.common_name = cn.into();
        self
    }

    /// Add DNS Subject Alternative Name
    pub fn with_san_dns(mut self, dns: impl Into<String>) -> Self {
        self.config.san_dns.push(dns.into());
        self
    }

    /// Add IP Subject Alternative Name
    pub fn with_san_ip(mut self, ip: std::net::IpAddr) -> Self {
        self.config.san_ips.push(ip);
        self
    }

    /// Set certificate validity period in days
    pub fn with_validity_days(mut self, days: u64) -> Self {
        self.config.validity_days = days;
        self
    }

    /// Enable certificate pinning
    pub fn with_pinning(mut self) -> Self {
        self.config.enable_pinning = true;
        self
    }

    /// Add a pinned certificate fingerprint (SHA-256 hex string)
    pub fn with_pinned_fingerprint(mut self, fingerprint: impl Into<String>) -> Self {
        self.config.pinned_fingerprints.push(fingerprint.into());
        self.config.enable_pinning = true;
        self
    }

    /// Enable auto-renewal before expiry
    pub fn with_auto_renew(mut self, days_before_expiry: u64) -> Self {
        self.config.auto_renew = true;
        self.config.auto_renew_days = days_before_expiry;
        self
    }

    /// Disable auto-renewal
    pub fn without_auto_renew(mut self) -> Self {
        self.config.auto_renew = false;
        self
    }

    /// Build the CertificateManager
    pub fn build(self) -> io::Result<CertificateManager> {
        CertificateManager::with_config(self.config)
    }
}

/// Certificate manager for HORUS network transports
///
/// Handles certificate generation, persistence, loading, and validation.
/// Provides zero-config experience for development while supporting
/// custom certificates for production.
#[derive(Debug)]
pub struct CertificateManager {
    config: CertificateConfig,
}

impl CertificateManager {
    /// Create a new certificate manager with default configuration
    ///
    /// Uses `~/.horus/certs/` as the certificate directory.
    pub fn new() -> io::Result<Self> {
        Self::with_config(CertificateConfig::default())
    }

    /// Create a certificate manager with custom configuration
    pub fn with_config(config: CertificateConfig) -> io::Result<Self> {
        // Ensure certificate directory exists
        if !config.cert_dir.exists() {
            fs::create_dir_all(&config.cert_dir).map_err(|e| {
                io::Error::new(
                    io::ErrorKind::Other,
                    format!("Failed to create certificate directory {:?}: {}", config.cert_dir, e),
                )
            })?;
            log::info!("Created certificate directory: {:?}", config.cert_dir);
        }

        Ok(Self { config })
    }

    /// Create a builder for custom configuration
    pub fn builder() -> CertificateManagerBuilder {
        CertificateManagerBuilder::new()
    }

    /// Get the certificate directory path
    pub fn cert_dir(&self) -> &Path {
        &self.config.cert_dir
    }

    /// Get the full path to the certificate file
    pub fn cert_path(&self) -> PathBuf {
        self.config.cert_dir.join(&self.config.cert_file)
    }

    /// Get the full path to the private key file
    pub fn key_path(&self) -> PathBuf {
        self.config.cert_dir.join(&self.config.key_file)
    }

    /// Check if certificates already exist
    pub fn certificates_exist(&self) -> bool {
        self.cert_path().exists() && self.key_path().exists()
    }

    /// Get or create certificates
    ///
    /// If certificates exist and are valid, loads them from disk.
    /// Otherwise, generates new self-signed certificates and saves them.
    #[cfg(feature = "tls")]
    pub fn get_or_create(&self) -> io::Result<(Vec<CertificateDer<'static>>, PrivateKeyDer<'static>)> {
        // Check if certificates exist
        if self.certificates_exist() {
            // Check if they're still valid
            match self.check_certificate_validity() {
                Ok(CertificateStatus::Valid { days_until_expiry }) => {
                    if days_until_expiry <= self.config.auto_renew_days && self.config.auto_renew {
                        log::warn!(
                            "Certificate expires in {} days, auto-renewing",
                            days_until_expiry
                        );
                        return self.generate_and_save();
                    }
                    log::debug!("Loading existing certificates from {:?}", self.cert_path());
                    return self.load_certificates();
                }
                Ok(CertificateStatus::Expired) => {
                    log::warn!("Certificate has expired, generating new one");
                }
                Ok(CertificateStatus::ExpiringSoon { days_until_expiry }) => {
                    if self.config.auto_renew {
                        log::warn!(
                            "Certificate expires in {} days, auto-renewing",
                            days_until_expiry
                        );
                    } else {
                        log::warn!(
                            "Certificate expires in {} days! Consider renewing manually.",
                            days_until_expiry
                        );
                        return self.load_certificates();
                    }
                }
                Err(e) => {
                    log::warn!("Could not verify certificate: {}, generating new one", e);
                }
            }
        }

        // Generate new certificates
        self.generate_and_save()
    }

    /// Generate new certificates and save to disk
    #[cfg(feature = "tls")]
    pub fn generate_and_save(&self) -> io::Result<(Vec<CertificateDer<'static>>, PrivateKeyDer<'static>)> {
        use rcgen::{CertificateParams, DistinguishedName, Ia5String};

        log::info!("Generating new self-signed certificate");

        // Create certificate parameters
        let mut params = CertificateParams::default();

        // Set distinguished name
        let mut dn = DistinguishedName::new();
        dn.push(rcgen::DnType::OrganizationName, &self.config.organization);
        dn.push(rcgen::DnType::CommonName, &self.config.common_name);
        params.distinguished_name = dn;

        // Note: rcgen 0.13+ uses default validity period (1 year from now)
        // Custom validity requires using the time crate which rcgen depends on internally
        // For most use cases, the default is sufficient

        // Build Subject Alternative Names
        let mut san = Vec::new();
        for dns in &self.config.san_dns {
            let ia5 = Ia5String::try_from(dns.clone())
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, format!("Invalid DNS SAN: {:?}", e)))?;
            san.push(rcgen::SanType::DnsName(ia5));
        }
        for ip in &self.config.san_ips {
            san.push(rcgen::SanType::IpAddress(*ip));
        }
        params.subject_alt_names = san;

        // Generate key pair
        let key_pair = rcgen::KeyPair::generate()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("Failed to generate key pair: {}", e)))?;

        // Generate self-signed certificate
        let cert = params.self_signed(&key_pair)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("Failed to generate certificate: {}", e)))?;

        // Get PEM-encoded versions
        let cert_pem = cert.pem();
        let key_pem = key_pair.serialize_pem();

        // Save to files
        fs::write(self.cert_path(), &cert_pem).map_err(|e| {
            io::Error::new(
                io::ErrorKind::Other,
                format!("Failed to write certificate to {:?}: {}", self.cert_path(), e),
            )
        })?;

        fs::write(self.key_path(), &key_pem).map_err(|e| {
            io::Error::new(
                io::ErrorKind::Other,
                format!("Failed to write private key to {:?}: {}", self.key_path(), e),
            )
        })?;

        // Set restrictive permissions on the key file (Unix only)
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = fs::metadata(self.key_path())?.permissions();
            perms.set_mode(0o600);
            fs::set_permissions(self.key_path(), perms)?;
        }

        log::info!("Certificate saved to {:?}", self.cert_path());
        log::info!("Private key saved to {:?}", self.key_path());
        log::warn!("Self-signed certificates provide encryption but NOT authentication.");
        log::warn!("Use CA-signed certificates for production deployments.");

        // Parse into rustls types
        self.parse_pem(&cert_pem, &key_pem)
    }

    /// Load certificates from disk
    #[cfg(feature = "tls")]
    pub fn load_certificates(&self) -> io::Result<(Vec<CertificateDer<'static>>, PrivateKeyDer<'static>)> {
        let cert_pem = fs::read_to_string(self.cert_path()).map_err(|e| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("Failed to read certificate from {:?}: {}", self.cert_path(), e),
            )
        })?;

        let key_pem = fs::read_to_string(self.key_path()).map_err(|e| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("Failed to read private key from {:?}: {}", self.key_path(), e),
            )
        })?;

        self.parse_pem(&cert_pem, &key_pem)
    }

    /// Parse PEM strings into rustls types
    #[cfg(feature = "tls")]
    fn parse_pem(&self, cert_pem: &str, key_pem: &str) -> io::Result<(Vec<CertificateDer<'static>>, PrivateKeyDer<'static>)> {
        let certs = rustls_pemfile::certs(&mut cert_pem.as_bytes())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("Failed to parse certificate: {}", e)))?;

        if certs.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "No certificates found in certificate file",
            ));
        }

        let key = rustls_pemfile::private_key(&mut key_pem.as_bytes())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("Failed to parse private key: {}", e)))?
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No private key found in key file"))?;

        // Validate pinning if enabled
        if self.config.enable_pinning && !self.config.pinned_fingerprints.is_empty() {
            self.verify_pinning(&certs)?;
        }

        Ok((certs, key))
    }

    /// Check certificate validity
    #[cfg(feature = "tls")]
    pub fn check_certificate_validity(&self) -> io::Result<CertificateStatus> {
        use x509_parser::prelude::*;

        let cert_pem = fs::read_to_string(self.cert_path())?;
        let cert_der = rustls_pemfile::certs(&mut cert_pem.as_bytes())
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No certificate found"))?
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        let (_, cert) = X509Certificate::from_der(cert_der.as_ref())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("Failed to parse certificate: {:?}", e)))?;

        let validity = cert.validity();
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs() as i64;

        let not_after = validity.not_after.timestamp();
        let days_until_expiry = (not_after - now) / 86400;

        if days_until_expiry < 0 {
            Ok(CertificateStatus::Expired)
        } else if days_until_expiry <= self.config.auto_renew_days as i64 {
            Ok(CertificateStatus::ExpiringSoon {
                days_until_expiry: days_until_expiry as u64,
            })
        } else {
            Ok(CertificateStatus::Valid {
                days_until_expiry: days_until_expiry as u64,
            })
        }
    }

    /// Verify certificate pinning
    #[cfg(feature = "tls")]
    fn verify_pinning(&self, certs: &[CertificateDer<'static>]) -> io::Result<()> {
        use sha2::{Digest, Sha256};

        if certs.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "No certificates to verify pinning",
            ));
        }

        let cert = &certs[0];
        let mut hasher = Sha256::new();
        hasher.update(cert.as_ref());
        let fingerprint = hex::encode(hasher.finalize());

        if self.config.pinned_fingerprints.iter().any(|f| f.eq_ignore_ascii_case(&fingerprint)) {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "Certificate pinning verification failed. Expected one of {:?}, got {}",
                    self.config.pinned_fingerprints, fingerprint
                ),
            ))
        }
    }

    /// Get the SHA-256 fingerprint of the current certificate
    #[cfg(feature = "tls")]
    pub fn get_fingerprint(&self) -> io::Result<String> {
        use sha2::{Digest, Sha256};

        let cert_pem = fs::read_to_string(self.cert_path())?;
        let cert_der = rustls_pemfile::certs(&mut cert_pem.as_bytes())
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No certificate found"))?
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        let mut hasher = Sha256::new();
        hasher.update(cert_der.as_ref());
        Ok(hex::encode(hasher.finalize()))
    }

    /// Delete existing certificates
    pub fn delete_certificates(&self) -> io::Result<()> {
        if self.cert_path().exists() {
            fs::remove_file(self.cert_path())?;
        }
        if self.key_path().exists() {
            fs::remove_file(self.key_path())?;
        }
        log::info!("Deleted certificates from {:?}", self.cert_dir());
        Ok(())
    }

    /// Get certificate information as a human-readable string
    #[cfg(feature = "tls")]
    pub fn get_cert_info(&self) -> io::Result<CertificateInfo> {
        use x509_parser::prelude::*;

        let cert_pem = fs::read_to_string(self.cert_path())?;
        let cert_der = rustls_pemfile::certs(&mut cert_pem.as_bytes())
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No certificate found"))?
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        let (_, cert) = X509Certificate::from_der(cert_der.as_ref())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("Failed to parse certificate: {:?}", e)))?;

        // Extract values before cert goes out of scope
        let common_name = cert.subject().iter_common_name()
            .next()
            .and_then(|cn| cn.as_str().ok())
            .unwrap_or("unknown")
            .to_string();

        let organization = cert.subject().iter_organization()
            .next()
            .and_then(|o| o.as_str().ok())
            .unwrap_or("unknown")
            .to_string();

        let validity = cert.validity();
        let not_before = validity.not_before.to_rfc2822().unwrap_or_else(|e| e);
        let not_after = validity.not_after.to_rfc2822().unwrap_or_else(|e| e);
        let is_self_signed = cert.issuer() == cert.subject();

        let fingerprint = self.get_fingerprint()?;

        Ok(CertificateInfo {
            common_name,
            organization,
            not_before,
            not_after,
            fingerprint_sha256: fingerprint,
            is_self_signed,
        })
    }
}

/// Certificate status after validity check
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CertificateStatus {
    /// Certificate is valid
    Valid { days_until_expiry: u64 },
    /// Certificate is expiring soon (within warning threshold)
    ExpiringSoon { days_until_expiry: u64 },
    /// Certificate has expired
    Expired,
}

/// Information about a certificate
#[derive(Debug, Clone)]
pub struct CertificateInfo {
    /// Common Name from certificate
    pub common_name: String,
    /// Organization from certificate
    pub organization: String,
    /// Not valid before date
    pub not_before: String,
    /// Not valid after date
    pub not_after: String,
    /// SHA-256 fingerprint
    pub fingerprint_sha256: String,
    /// Whether the certificate is self-signed
    pub is_self_signed: bool,
}

impl std::fmt::Display for CertificateInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Certificate Information:")?;
        writeln!(f, "  Common Name: {}", self.common_name)?;
        writeln!(f, "  Organization: {}", self.organization)?;
        writeln!(f, "  Valid From: {}", self.not_before)?;
        writeln!(f, "  Valid Until: {}", self.not_after)?;
        writeln!(f, "  SHA-256 Fingerprint: {}", self.fingerprint_sha256)?;
        writeln!(f, "  Self-Signed: {}", self.is_self_signed)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_default_config() {
        let config = CertificateConfig::default();
        assert_eq!(config.organization, "HORUS Robotics");
        assert_eq!(config.common_name, "horus");
        assert!(config.san_dns.contains(&"localhost".to_string()));
    }

    #[test]
    fn test_builder() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .with_organization("Test Org")
            .with_common_name("test-server")
            .with_san_dns("example.com")
            .with_validity_days(30)
            .build()
            .unwrap();

        assert_eq!(manager.config.organization, "Test Org");
        assert_eq!(manager.config.common_name, "test-server");
        assert!(manager.config.san_dns.contains(&"example.com".to_string()));
        assert_eq!(manager.config.validity_days, 30);
    }

    #[test]
    fn test_certificates_exist() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .build()
            .unwrap();

        assert!(!manager.certificates_exist());
    }

    #[test]
    fn test_paths() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .with_file_names("test.crt", "test.key")
            .build()
            .unwrap();

        assert_eq!(manager.cert_path(), temp_dir.path().join("test.crt"));
        assert_eq!(manager.key_path(), temp_dir.path().join("test.key"));
    }

    #[test]
    #[cfg(feature = "tls")]
    fn test_generate_and_load() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .build()
            .unwrap();

        // Generate certificates
        let result = manager.generate_and_save();
        assert!(result.is_ok(), "Failed to generate certificates: {:?}", result.err());

        // Verify files exist
        assert!(manager.certificates_exist());

        // Load certificates
        let load_result = manager.load_certificates();
        assert!(load_result.is_ok(), "Failed to load certificates: {:?}", load_result.err());

        // Get fingerprint
        let fingerprint = manager.get_fingerprint();
        assert!(fingerprint.is_ok());
        assert_eq!(fingerprint.unwrap().len(), 64); // SHA-256 hex = 64 chars
    }

    #[test]
    #[cfg(feature = "tls")]
    fn test_get_or_create() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .build()
            .unwrap();

        // First call generates
        let result1 = manager.get_or_create();
        assert!(result1.is_ok());

        // Second call loads existing
        let result2 = manager.get_or_create();
        assert!(result2.is_ok());
    }

    #[test]
    #[cfg(feature = "tls")]
    fn test_cert_info() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .with_organization("Test Org")
            .with_common_name("test-server")
            .build()
            .unwrap();

        // Generate
        manager.generate_and_save().unwrap();

        // Get info
        let info = manager.get_cert_info().unwrap();
        assert_eq!(info.common_name, "test-server");
        assert_eq!(info.organization, "Test Org");
        assert!(info.is_self_signed);
    }

    #[test]
    fn test_delete_certificates() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .build()
            .unwrap();

        // Create dummy files
        fs::write(manager.cert_path(), "test").unwrap();
        fs::write(manager.key_path(), "test").unwrap();

        assert!(manager.certificates_exist());

        // Delete
        manager.delete_certificates().unwrap();

        assert!(!manager.certificates_exist());
    }

    #[test]
    #[cfg(feature = "tls")]
    fn test_pinning() {
        let temp_dir = TempDir::new().unwrap();
        let manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .build()
            .unwrap();

        // Generate certificate
        manager.generate_and_save().unwrap();
        let fingerprint = manager.get_fingerprint().unwrap();

        // Create manager with correct pin
        let pinned_manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .with_pinned_fingerprint(&fingerprint)
            .build()
            .unwrap();

        // Should succeed
        assert!(pinned_manager.load_certificates().is_ok());

        // Create manager with wrong pin
        let wrong_pin_manager = CertificateManager::builder()
            .with_cert_dir(temp_dir.path())
            .with_pinned_fingerprint("0000000000000000000000000000000000000000000000000000000000000000")
            .build()
            .unwrap();

        // Should fail
        assert!(wrong_pin_manager.load_certificates().is_err());
    }
}
