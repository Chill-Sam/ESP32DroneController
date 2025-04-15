use hex;
use hmac::{Hmac, Mac};
use rand::distributions::Alphanumeric;
use rand::{Rng, thread_rng};
use serde::Deserialize;
use sha2::Sha256;
use std::env;
use subtle::ConstantTimeEq;

type HmacSha256 = Hmac<Sha256>;

#[derive(Debug, Deserialize)]
pub struct AuthMessage {
    pub role: String,
    pub signature: String,
}

pub fn generate_nonce() -> String {
    thread_rng()
        .sample_iter(&Alphanumeric)
        .take(32)
        .map(char::from)
        .collect()
}

pub fn validate_signature(role: &str, nonce: &str, signature_hex: &str) -> bool {
    let secret = match role {
        "drone" => env::var("DRONE_SECRET").unwrap_or_default(),
        "controller" => env::var("CONTROLLER_SECRET").unwrap_or_default(),
        _ => return false,
    };

    let mut mac = HmacSha256::new_from_slice(secret.as_bytes()).unwrap();
    mac.update(nonce.as_bytes());
    let expected = mac.finalize().into_bytes();

    if let Ok(provided) = hex::decode(signature_hex) {
        provided.ct_eq(&expected).into()
    } else {
        false
    }
}
