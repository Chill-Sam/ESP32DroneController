use actix_web::{Error, HttpRequest, HttpResponse, web};
use actix_ws::{Message, Session, handle};
use futures_util::StreamExt;
use once_cell::sync::Lazy;
use std::collections::HashMap;
use std::sync::Mutex;

use crate::auth::generate_nonce;
use crate::handlers::{handle_auth, send_nonce};
use crate::message::MessageTypeOnly;

// Thread-safe global client registry
static CLIENTS: Lazy<Mutex<HashMap<String, Session>>> = Lazy::new(|| Mutex::new(HashMap::new()));

pub async fn echo(req: HttpRequest, body: web::Payload) -> Result<HttpResponse, Error> {
    let (response, mut session, mut stream) = handle(&req, body)?;
    println!("WebSocket connection received");

    let nonce = generate_nonce();
    send_nonce(&mut session, &nonce).await;
    println!("Sent nonce: {}", nonce);

    actix_web::rt::spawn(async move {
        let mut authed_role: Option<String> = None;

        while let Some(Ok(msg)) = stream.next().await {
            let Message::Text(text) = msg else { continue };

            // AUTH FLOW
            if authed_role.is_none() {
                if let Some(auth) = handle_auth(&text, &nonce, &mut session).await {
                    println!("Authenticated: {}", auth.role);
                    {
                        let mut clients = CLIENTS.lock().unwrap();
                        clients.insert(auth.role.clone(), session.clone());
                    }
                    authed_role = Some(auth.role);
                } else {
                    break;
                }
                continue;
            }

            // MESSAGE FORWARDING
            let role = authed_role.as_ref().unwrap();
            let target = if role == "controller" {
                "drone"
            } else {
                "controller"
            };

            let msg_type = match serde_json::from_str::<MessageTypeOnly>(&text) {
                Ok(mt) => mt.r#type.to_lowercase(),
                Err(_) => {
                    let _ = session.text(r#"{"error":"Invalid message format"}"#).await;
                    continue;
                }
            };

            match (role.as_str(), msg_type.as_str()) {
                ("controller", "command") | ("controller", "pid") | ("drone", "telemetry") => {
                    let clients = CLIENTS.lock().unwrap();
                    if let Some(target_session) = clients.get(target) {
                        let mut target = target_session.clone();
                        let _ = target.text(text.clone()).await;
                    } else {
                        let _ = session
                            .text(format!(r#"{{"error":"{} not connected"}}"#, target))
                            .await;
                    }
                }
                _ => {
                    let _ = session
                        .text(r#"{"error":"Message type not allowed for role"}"#)
                        .await;
                }
            }
        }

        // Clean up
        if let Some(role) = authed_role {
            println!("Cleaning up client: {}", role);
            let mut clients = CLIENTS.lock().unwrap();
            clients.remove(&role);
        }

        println!("Client session ended");
    });
    Ok(response)
}
