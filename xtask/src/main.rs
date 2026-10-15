//! Repository tasks.
//!
//! Workspace-level checks as ordinary Rust — compiled, linted and tested —
//! rather than shell in the CI file.
//!
//! ```text
//! cargo xtask <task>
//! ```

mod arch;
mod mavlink;

use std::process::ExitCode;

fn main() -> ExitCode {
    let task = std::env::args().nth(1);
    match task.as_deref() {
        Some("help") | None => {
            help();
            ExitCode::SUCCESS
        }
        Some("arch") => match arch::run() {
            Ok(()) => ExitCode::SUCCESS,
            Err(failure) => {
                eprintln!("{failure}");
                ExitCode::FAILURE
            }
        },
        Some("mavlink") => {
            let args: Vec<String> = std::env::args().skip(2).collect();
            match mavlink::run(&args) {
                Ok(()) => ExitCode::SUCCESS,
                Err(failure) => {
                    eprintln!("{failure}");
                    ExitCode::FAILURE
                }
            }
        }
        Some(unknown) => {
            eprintln!("unknown task: {unknown}\n");
            help();
            ExitCode::FAILURE
        }
    }
}

fn help() {
    eprintln!("Tasks:");
    eprintln!("  arch   check the dependency rule against ci/allowed-deps.toml");
    eprintln!("  mavlink DEFINITIONS.xml [LABEL]");
    eprintln!("         generate the messages of kinavis-mavlink from a MAVLink dialect");
    eprintln!("  help   this message");
}
