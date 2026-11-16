//! Local single-instance ownership and an acknowledged "show window" request.
//! This listener binds loopback only. It never carries tracker or pose data.
use std::{
    io::{self, Read, Write},
    net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

const PORT: u16 = 39572;
const REQUEST: &[u8] = b"CHADFBT SHOW WINDOW 1\n";
const ACK: &[u8] = b"CHADFBT SHOW ACCEPTED 1\n";
const BUSY: &[u8] = b"CHADFBT HEADLESS SESSION\n";
const TIMEOUT: Duration = Duration::from_millis(600);
static SHOW_REQUESTED: AtomicBool = AtomicBool::new(false);
type Waker = Arc<dyn Fn() + Send + Sync>;
static WAKER: Mutex<Option<Waker>> = Mutex::new(None);

pub enum Instance {
    Primary(Guard),
    /// A primary instance positively acknowledged this exact protocol request.
    Restored,
}

pub struct Guard {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

/// Acquire ownership before constructing the runtime or opening its UDP socket.
/// A port collision only counts as an existing ChadFBT after a valid ACK.
pub fn acquire(ui_expected: bool) -> io::Result<Instance> {
    let address = SocketAddr::from((Ipv4Addr::LOCALHOST, PORT));
    match TcpListener::bind(address) {
        Ok(listener) => start_listener(listener, ui_expected).map(Instance::Primary),
        Err(error) if error.kind() == io::ErrorKind::AddrInUse => {
            if !ui_expected {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    "ChadFBT or another service already owns the local session port; no second tracker receiver was started",
                ));
            }
            notify_existing(address)?;
            Ok(Instance::Restored)
        }
        Err(error) => Err(error),
    }
}

fn start_listener(listener: TcpListener, ui_expected: bool) -> io::Result<Guard> {
    listener.set_nonblocking(true)?;
    let stop = Arc::new(AtomicBool::new(false));
    let worker_stop = stop.clone();
    let worker = thread::Builder::new()
        .name("chadfbt-window-requests".to_owned())
        .spawn(move || {
            while !worker_stop.load(Ordering::Acquire) {
                match listener.accept() {
                    Ok((stream, peer)) if peer.ip().is_loopback() => {
                        let _ = handle_request(stream, ui_expected);
                    }
                    Ok(_) => {}
                    Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(20));
                    }
                    // Transient accept failures (aborted connection, resource
                    // pressure) must not release the single-instance port.
                    Err(_) => thread::sleep(Duration::from_millis(100)),
                }
            }
        })?;
    Ok(Guard {
        stop,
        worker: Some(worker),
    })
}

fn configure(stream: &TcpStream) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(TIMEOUT))?;
    stream.set_write_timeout(Some(TIMEOUT))
}

fn handle_request(mut stream: TcpStream, ui_expected: bool) -> io::Result<()> {
    configure(&stream)?;
    let mut request = [0; REQUEST.len()];
    stream.read_exact(&mut request)?;
    if request != REQUEST {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unrecognized local window request",
        ));
    }
    if !ui_expected {
        stream.write_all(BUSY)?;
        // Let the client close first, so the listening port can be reclaimed
        // immediately after a normal app shutdown on Windows.
        let _ = stream.read(&mut [0; 1]);
        return Ok(());
    }
    SHOW_REQUESTED.store(true, Ordering::Release);
    if let Some(wake) = WAKER.lock().unwrap_or_else(|e| e.into_inner()).clone() {
        wake();
    }
    stream.write_all(ACK)?;
    let _ = stream.read(&mut [0; 1]);
    Ok(())
}

fn notify_existing(address: SocketAddr) -> io::Result<()> {
    let mut stream = TcpStream::connect_timeout(&address, TIMEOUT)?;
    configure(&stream)?;
    stream.write_all(REQUEST)?;
    let mut acknowledgment = [0; ACK.len()];
    stream.read_exact(&mut acknowledgment)?;
    if acknowledgment != ACK {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Local port is occupied but no ChadFBT window acknowledgment was received; no second receiver was started",
        ));
    }
    Ok(())
}

/// Register the GUI event-loop wake callback, including during hidden operation.
pub fn set_waker(waker: Waker) {
    *WAKER.lock().unwrap_or_else(|e| e.into_inner()) = Some(waker.clone());
    if SHOW_REQUESTED.load(Ordering::Acquire) {
        waker();
    }
}

pub fn take_show_request() -> bool {
    SHOW_REQUESTED.swap(false, Ordering::AcqRel)
}

impl Drop for Guard {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    static SERIAL: Mutex<()> = Mutex::new(());

    #[test]
    fn acknowledged_restore_sets_show_request() {
        let _serial = SERIAL.lock().unwrap();
        take_show_request();
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let address = listener.local_addr().unwrap();
        let _guard = start_listener(listener, true).unwrap();
        notify_existing(address).unwrap();
        assert!(take_show_request());
        assert!(!take_show_request());
    }

    #[test]
    fn another_service_cannot_silently_exit_the_second_app() {
        let _serial = SERIAL.lock().unwrap();
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let address = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            configure(&stream).unwrap();
            let mut request = [0; REQUEST.len()];
            stream.read_exact(&mut request).unwrap();
            stream.write_all(&[b'X'; ACK.len()]).unwrap();
        });
        assert_eq!(
            notify_existing(address).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        server.join().unwrap();
    }

    #[test]
    fn headless_instance_cannot_claim_to_show_a_window() {
        let _serial = SERIAL.lock().unwrap();
        take_show_request();
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let address = listener.local_addr().unwrap();
        let _guard = start_listener(listener, false).unwrap();
        assert!(notify_existing(address).is_err());
        assert!(!take_show_request());
    }

    #[test]
    fn shutdown_releases_port_for_immediate_relaunch() {
        let _serial = SERIAL.lock().unwrap();
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let address = listener.local_addr().unwrap();
        let guard = start_listener(listener, true).unwrap();
        notify_existing(address).unwrap();
        drop(guard);
        let _replacement = TcpListener::bind(address).unwrap();
        take_show_request();
    }
}
