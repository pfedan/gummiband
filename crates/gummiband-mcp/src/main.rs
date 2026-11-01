//! `gummiband-mcp`: Model Context Protocol server (stdio) for the Gummiband autorouter.
//!
//! AI agents load KiCad boards into a session, inspect nets, set per net options, route,
//! run the KiCad DRC and render PNG images of board regions. See docs/MCP.md.

mod external;
mod practice;
mod server;

use anyhow::Result;
use rmcp::ServiceExt;

/// Rayon threads when RAYON_NUM_THREADS is unset: the machine is shared with other jobs.
const DEFAULT_THREADS: usize = 4;

fn main() -> Result<()> {
    // Rayon reads RAYON_NUM_THREADS itself when it builds the global pool; only pick our own
    // default when the variable is missing or invalid.
    let from_env = std::env::var("RAYON_NUM_THREADS")
        .ok()
        .and_then(|v| v.trim().parse::<usize>().ok())
        .filter(|&n| n > 0);
    let threads = from_env.unwrap_or(DEFAULT_THREADS);
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .thread_name(|i| format!("gummiband-rayon-{i}"))
        .build_global()?;
    eprintln!(
        "gummiband-mcp {}: stdio MCP server, {} routing threads",
        env!("CARGO_PKG_VERSION"),
        rayon::current_num_threads()
    );

    // A small async runtime for the protocol; routing, DRC and rendering run on blocking
    // threads (routing fans out on the rayon pool above).
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    rt.block_on(async {
        let service = server::Gummiband::new()
            .serve(rmcp::transport::stdio())
            .await
            .map_err(|e| anyhow::anyhow!("MCP handshake failed: {e}"))?;
        service.waiting().await?;
        Ok(())
    })
}
