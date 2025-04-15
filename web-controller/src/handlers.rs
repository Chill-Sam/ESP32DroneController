use crate::auth::{AuthMessage, validate_signature};
use actix_ws::Session;
use serde_json;

pub async fn send_nonce(session: &mut Session, nonce: &str) {
    let msg = serde_json::json!({ "nonce": nonce }).to_string();
    let _ = session.text(msg).await;
}

pub async fn handle_auth(text: &str, nonce: &str, session: &mut Session) -> Option<AuthMessage> {
    let Ok(auth) = serde_json::from_str::<AuthMessage>(text) else {
        let _ = session.text(r#"{"error":"Malformed auth"}"#).await;
        return None;
    };

    if !validate_signature(&auth.role, nonce, &auth.signature) {
        let _ = session.text(r#"{"error":"Invalid signature"}"#).await;
        return None;
    }

    let _ = session.text(r#"{"status":"ok"}"#).await;
    Some(auth)
}
