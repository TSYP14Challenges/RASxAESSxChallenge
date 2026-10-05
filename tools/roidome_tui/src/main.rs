mod app;
mod frame;
mod serial;
mod ui;

use app::App;
use crossterm::event::{self, Event, KeyCode};
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let Some(port) = args.get(1).cloned() else {
        eprintln!("usage: {} <bridge serial port> [baud=115200]", args[0]);
        eprintln!("       {} --replay <saved bridge log>", args[0]);
        eprintln!("available ports:");
        for p in serialport::available_ports().unwrap_or_default() {
            eprintln!("  {}", p.port_name);
        }
        std::process::exit(2);
    };

    if port == "--replay" {
        let path = args.get(2).cloned().ok_or("--replay needs a file")?;
        let app = Arc::new(Mutex::new(App::new(format!("replay {path}"))));
        serial::start_replay(Arc::clone(&app), path);
        return run_tui(app);
    }

    let baud: u32 = args.get(2).map(|b| b.parse()).transpose()?.unwrap_or(115200);
    let app = Arc::new(Mutex::new(App::new(port.clone())));

    // Serial reader thread (replaces the MQTT task)
    serial::start_serial(Arc::clone(&app), port, baud);
    serial::start_injector(Arc::clone(&app));
    run_tui(app)
}

fn run_tui(app: Arc<Mutex<App>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut terminal = ratatui::init();
    let app_tui = Arc::clone(&app);

    loop {
        {
            let mut app = app_tui.lock().unwrap();
            if !app.running {
                break;
            }
            terminal.draw(|frame| {
                ui::ui(frame, &mut app);
            })?;
        }

        if event::poll(Duration::from_millis(250))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') => {
                        app_tui.lock().unwrap().running = false;
                        break;
                    }
                    KeyCode::Char('c') => app_tui.lock().unwrap().clear(),
                    _ => {}
                }
            }
        }
    }

    ratatui::restore();
    Ok(())
}
