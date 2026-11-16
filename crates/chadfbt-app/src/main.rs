#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod analysis;
mod audio_cues;
mod calibration_request;
mod identification;
mod model;
mod output_mode;
mod outputs;
mod replay;
mod runtime;
mod single_instance;
mod tap_gesture;
mod transport;
mod ui;
mod voice_prompts;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!(
            "ChadFBT experimental native tracking workbench\n\n  chadfbt                       Native UI\n  chadfbt --headless --seconds N Capture health without a window\n  chadfbt --replay FILE          Replay a JSONL recording without network output\n  chadfbt --replay FILE --dump OUT.jsonl [--every N] [--estimator JSON|@FILE] [--raw-dimensions]\n                               Re-solve the recorded inputs with the current code and\n                               write every N-th solved frame; model output, not accuracy\n  chadfbt --analyze-heading FILE --report NEW.json\n                               Offline model-only heading analysis; no VR output\n\nProfiles: %LOCALAPPDATA%\\ChadFBT\\profile.json\nOutput mode (saved; default Auto): live in SteamVR with a worn headset and a valid\ncalibration, desktop preview otherwise. Nothing is sent during calibration."
        );
        return;
    }
    if let Some(index) = args
        .iter()
        .position(|argument| argument == "--analyze-heading")
    {
        let input = args.get(index + 1).filter(|value| !value.starts_with("--"));
        let output = args
            .iter()
            .position(|argument| argument == "--report")
            .and_then(|index| args.get(index + 1))
            .filter(|value| !value.starts_with("--"));
        let (Some(input), Some(output)) = (input, output) else {
            eprintln!("Usage: chadfbt --analyze-heading RECORDING.jsonl --report NEW-REPORT.json");
            std::process::exit(2);
        };
        match analysis::analyze_recording(std::path::Path::new(input), std::path::Path::new(output))
        {
            Ok(report) => println!("{}", analysis::summary(&report)),
            Err(error) => {
                eprintln!("Offline heading analysis failed: {error}");
                std::process::exit(1);
            }
        }
        return;
    }
    if let Some(i) = args.iter().position(|arg| arg == "--replay") {
        let value = |flag: &str| {
            args.iter()
                .position(|argument| argument == flag)
                .and_then(|index| args.get(index + 1))
                .filter(|value| !value.starts_with("--"))
        };
        if let Some(output) = value("--dump") {
            let Some(input) = args.get(i + 1).filter(|value| !value.starts_with("--")) else {
                eprintln!("--replay requires a recording path");
                std::process::exit(2);
            };
            let every = match value("--every").map(|n| n.parse::<u32>()) {
                None => 1,
                Some(Ok(n)) if n >= 1 => n,
                Some(_) => {
                    eprintln!("--every expects a positive integer");
                    std::process::exit(2);
                }
            };
            let estimator_overrides = match value("--estimator") {
                None => None,
                Some(text) => {
                    let text = match text.strip_prefix('@') {
                        Some(path) => std::fs::read_to_string(path).unwrap_or_else(|error| {
                            eprintln!("--estimator {path}: {error}");
                            std::process::exit(2);
                        }),
                        None => text.clone(),
                    };
                    match serde_json::from_str::<serde_json::Value>(&text) {
                        Ok(value) if value.is_object() => Some(value),
                        _ => {
                            eprintln!("--estimator expects a JSON object");
                            std::process::exit(2);
                        }
                    }
                }
            };
            let options = replay::DumpOptions {
                every,
                estimator_overrides,
                raw_dimensions: args.iter().any(|argument| argument == "--raw-dimensions"),
            };
            match replay::dump(
                std::path::Path::new(input),
                std::path::Path::new(output),
                &options,
            ) {
                Ok(summary) => println!(
                    "Re-solved {} frames ({} written, {} events, {} failed, {} ingest outcome differences); solve time median {:.0} us, p99 {:.0} us. Model output from recorded inputs, not accuracy. No network output was sent.",
                    summary.solves,
                    summary.written,
                    summary.events,
                    summary.failed_events,
                    summary.outcome_mismatches,
                    summary.solve_micros_median,
                    summary.solve_micros_p99
                ),
                Err(error) => {
                    eprintln!("Replay dump failed: {error}");
                    std::process::exit(1);
                }
            }
            return;
        }
        match args.get(i + 1) {
            Some(path) => {
                if let Err(err) = runtime::replay(std::path::Path::new(path)) {
                    eprintln!("Replay failed: {err}");
                    std::process::exit(1);
                }
            }
            None => {
                eprintln!("--replay requires a recording path");
                std::process::exit(2);
            }
        }
        return;
    }
    let headless = args.iter().any(|arg| arg == "--headless");
    let _instance = match single_instance::acquire(!headless) {
        Ok(single_instance::Instance::Primary(guard)) => guard,
        Ok(single_instance::Instance::Restored) => return,
        Err(error) => {
            eprintln!("Could not open ChadFBT session: {error}");
            std::process::exit(1);
        }
    };
    let (handle, worker) = runtime::start();
    if args.iter().any(|arg| arg == "--headless") {
        let seconds = args
            .iter()
            .position(|arg| arg == "--seconds")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(30)
            .clamp(1, 3600);
        for _ in 0..seconds {
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
        let status = handle.worker_status();
        if status != model::WorkerStatus::Running {
            eprintln!("Tracking worker is not running ({status:?}); the snapshot below is stale.");
        }
        println!(
            "{}",
            serde_json::to_string_pretty(&handle.snapshot()).unwrap()
        );
    } else if let Err(err) = ui::run(handle.clone()) {
        eprintln!("Native UI failed: {err}");
    }
    handle.send(model::Command::Shutdown);
    let _ = worker.join();
}
