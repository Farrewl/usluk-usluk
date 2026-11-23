use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    routing::get,
    Router,
};
use dashmap::DashMap;
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::broadcast;
use tower_http::cors::CorsLayer;
use tracing::{info, warn};

#[derive(Debug, Serialize, Deserialize, Clone)]
struct Envelope {
    v: u32,
    kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    seq: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    t_unix: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    boat_id: Option<String>,
    #[serde(flatten)]
    payload: serde_json::Value,
}

struct AppState {
    tx: broadcast::Sender<Message>,
    last_telemetry: DashMap<String, serde_json::Value>,
    last_params: DashMap<String, serde_json::Value>,
    publisher_token: String,
    admin_token: String,
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    let publisher_token = std::env::var("PUBLISHER_TOKEN").unwrap_or_else(|_| "pub_secret".to_string());
    let admin_token = std::env::var("ADMIN_TOKEN").unwrap_or_else(|_| "admin_secret".to_string());

    let (tx, _rx) = broadcast::channel(100);
    let state = Arc::new(AppState {
        tx,
        last_telemetry: DashMap::new(),
        last_params: DashMap::new(),
        publisher_token,
        admin_token,
    });

    let app = Router::new()
        .route("/ws", get(ws_handler))
        .route("/healthz", get(|| async { "ok" }))
        .layer(CorsLayer::permissive())
        .with_state(state);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    info!("Relay listening on 0.0.0.0:3000");
    axum::serve(listener, app).await.unwrap();
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<Arc<AppState>>) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: Arc<AppState>) {
    let (mut sender, mut receiver) = socket.split();
    let mut rx = state.tx.subscribe();
    let mut role = "viewer".to_string();
    let mut authenticated = false;

    // Send last known state to new viewer
    for entry in state.last_telemetry.iter() {
        let _ = sender.send(Message::Text(entry.value().to_string().into())).await;
    }

    let tx = state.tx.clone();
    let state_task = Arc::clone(&state);

    let mut recv_task = tokio::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            if sender.send(msg).await.is_err() {
                break;
            }
        }
    });

    while let Some(Ok(msg)) = receiver.next().await {
        match msg {
            Message::Text(text) => {
                if let Ok(envelope) = serde_json::from_str::<Envelope>(&text) {
                    if envelope.kind == "hello" {
                        let token = envelope.payload.get("token").and_then(|v| v.as_str()).unwrap_or("");
                        if token == state_task.publisher_token {
                            role = "publisher".to_string();
                            authenticated = true;
                            info!("Publisher connected");
                        } else if token == state_task.admin_token {
                            role = "admin".to_string();
                            authenticated = true;
                            info!("Admin connected");
                        } else if token.is_empty() {
                            // Viewer connects without token; read-only access.
                            info!("Viewer connected");
                        }
                    }

                    if authenticated && (role == "publisher" || role == "admin") {
                        if envelope.kind == "telemetry" {
                            state_task.last_telemetry.insert("latest".to_string(), serde_json::to_value(&envelope).unwrap());
                        } else if envelope.kind == "params_snapshot" {
                            state_task.last_params.insert("latest".to_string(), serde_json::to_value(&envelope).unwrap());
                        }
                    }

                    let _ = tx.send(Message::Text(text.into())).await;
                }
            }
            Message::Binary(bin) => {
                let _ = tx.send(Message::Binary(bin)).await;
            }
            _ => {}
        }
    }

    recv_task.abort();
}
