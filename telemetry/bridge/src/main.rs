use anyhow::Result;
use futures_util::{SinkExt, StreamExt};
use serde::Serialize;
use std::path::Path;
use std::time::Duration;
use tokio::fs;
use tokio::time::sleep;
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message};
use url::Url;

#[derive(Serialize)]
struct Hello {
    v: u32,
    kind: String,
    role: String,
    token: String,
}

#[derive(Serialize)]
struct Envelope<T> {
    v: u32,
    kind: String,
    seq: u64,
    t_unix: f64,
    boat_id: String,
    #[serde(flatten)]
    payload: T,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    let relay_url = std::env::var("RELAY_URL").unwrap_or_else(|_| "ws://127.0.0.1:3000/ws".to_string());
    let pub_token = std::env::var("PUBLISHER_TOKEN").unwrap_or_else(|_| "pub_secret".to_string());
    let monitor_dir = std::env::var("MONITOR_DIR").unwrap_or_else(|_| "data/monitor".to_string());

    let mut seq = 0u64;

    loop {
        println!("Connecting to relay: {}", relay_url);
        let url = Url::parse(&relay_url)?;
        
        match connect_async(url).await {
            Ok((ws_stream, _)) => {
                println!("Connected to relay!");
                let (mut write, mut read) = ws_stream.split();

                // Send hello
                let hello = Hello {
                    v: 1,
                    kind: "hello".into(),
                    role: "publisher".into(),
                    token: pub_token.clone(),
                };
                let _ = write.send(Message::Text(serde_json::to_string(&hello)?)).await;

                let mut last_telemetry_mod = std::time::SystemTime::UNIX_EPOCH;
                let mut last_frame_mod = std::time::SystemTime::UNIX_EPOCH;

                loop {
                    // Check telemetry
                    let telem_path = Path::new(&monitor_dir).join("telemetry.json");
                    if telem_path.exists() {
                        if let Ok(metadata) = fs::metadata(&telem_path).await {
                            if let Ok(modified) = metadata.modified() {
                                if modified > last_telemetry_mod {
                                    last_telemetry_mod = modified;
                                    if let Ok(content) = fs::read_to_string(&telem_path).await {
                                        if let Ok(json_val) = serde_json::from_str::<serde_json::Value>(&content) {
                                            seq += 1;
                                            let env = Envelope {
                                                v: 1,
                                                kind: "telemetry".into(),
                                                seq,
                                                t_unix: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64(),
                                                boat_id: "aterkia_asv".into(),
                                                payload: json_val,
                                            };
                                            if write.send(Message::Text(serde_json::to_string(&env)?)).await.is_err() {
                                                break; // Disconnected
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Check frame.jpg (Binary media format: 1 byte channel + 4 bytes seq + 8 bytes timestamp + jpeg bytes)
                    let frame_path = Path::new(&monitor_dir).join("frame.jpg");
                    if frame_path.exists() {
                        if let Ok(metadata) = fs::metadata(&frame_path).await {
                            if let Ok(modified) = metadata.modified() {
                                if modified > last_frame_mod {
                                    last_frame_mod = modified;
                                    if let Ok(jpeg_bytes) = fs::read(&frame_path).await {
                                        seq += 1;
                                        let mut packet = Vec::with_capacity(1 + 4 + 8 + jpeg_bytes.len());
                                        packet.push(0u8); // channel 0: nav
                                        packet.extend_from_slice(&(seq as u32).to_be_bytes());
                                        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
                                        packet.extend_from_slice(&now.to_be_bytes());
                                        packet.extend_from_slice(&jpeg_bytes);

                                        if write.send(Message::Binary(packet)).await.is_err() {
                                            break; // Disconnected
                                        }
                                    }
                                }
                            }
                        }
                    }

                    sleep(Duration::from_millis(100)).await;
                }
            }
            Err(e) => {
                println!("Failed to connect: {}. Retrying in 3s...", e);
                sleep(Duration::from_secs(3)).await;
            }
        }
    }
}
