use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct MessageTypeOnly {
    #[serde(rename = "type")]
    pub r#type: String,
}
