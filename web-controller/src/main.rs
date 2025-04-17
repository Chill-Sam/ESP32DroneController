mod auth;
mod handlers;
mod message;
mod ws;

use actix_files::Files;
use actix_web::{App, HttpServer, web};
use dotenv::dotenv;
use openssl::ssl::{SslAcceptor, SslFiletype, SslMethod};
use ws::echo;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    dotenv().ok();

    // Configure SSL
    let mut builder = SslAcceptor::mozilla_intermediate(SslMethod::tls()).unwrap();
    builder
        .set_private_key_file("certs/key.pem", SslFiletype::PEM)
        .unwrap();
    builder
        .set_certificate_chain_file("certs/cert.pem")
        .unwrap();

    HttpServer::new(|| {
        App::new()
            .route("/echo", web::get().to(echo))
            .service(Files::new("/", "./static").index_file("index.html"))
    })
    .bind_openssl("0.0.0.0:8443", builder)?
    .run()
    .await
}
