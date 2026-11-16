//! Calibration and gesture feedback on a dedicated audio thread, never the
//! tracking thread.
//!
//! Prompts combine short tones (punctuation, Windows `Beep`) with spoken words
//! from the Windows speech synthesizer (WinRT `SpeechSynthesizer`, played by a
//! WinRT `MediaPlayer` on the default audio render device). The worker is fed by
//! a small bounded channel; stale prompts are dropped instead of queued, a new
//! important prompt interrupts the current sentence, and a generation change
//! (cancel) silences everything queued or playing. If synthesis is unavailable
//! the prompt's tones (or a fallback tone) play instead. Audible delivery still
//! depends on the user's audio routing and volume.
use serde::{Deserialize, Serialize};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicU8, AtomicU64, Ordering},
    mpsc::{Receiver, RecvTimeoutError, SyncSender, sync_channel},
};
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cue {
    Countdown,
    Ready,
    /// Start of the movement phase of a movement calibration.
    Move,
    Success,
    Failure,
    Cancelled,
}

/// What the user hears; persisted in the app settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AudioMode {
    #[default]
    VoiceAndTones,
    TonesOnly,
    Silent,
}

impl AudioMode {
    pub const ALL: [AudioMode; 3] = [Self::VoiceAndTones, Self::TonesOnly, Self::Silent];

    pub fn label(self) -> &'static str {
        match self {
            Self::VoiceAndTones => "Voice + tones",
            Self::TonesOnly => "Tones only",
            Self::Silent => "Silent",
        }
    }

    fn to_u8(self) -> u8 {
        match self {
            Self::VoiceAndTones => 0,
            Self::TonesOnly => 1,
            Self::Silent => 2,
        }
    }

    fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::TonesOnly,
            2 => Self::Silent,
            _ => Self::VoiceAndTones,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Urgency {
    /// Plays only when nothing is being spoken (countdown ticks).
    Background,
    /// Stops the current sentence and supersedes older queued prompts.
    Interrupt,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Prompt {
    pub cue: Option<Cue>,
    pub text: Option<String>,
    /// Played instead of `text` when speech is off or unavailable and there is
    /// no `cue`.
    pub fallback: Option<Cue>,
    pub urgency: Urgency,
}

impl Prompt {
    pub fn tone(cue: Cue) -> Self {
        Self {
            cue: Some(cue),
            text: None,
            fallback: None,
            urgency: Urgency::Background,
        }
    }

    /// Tones (if any) followed by words; interrupts what is playing.
    pub fn say(cue: Option<Cue>, text: impl Into<String>) -> Self {
        Self {
            cue,
            text: Some(text.into()),
            fallback: Some(cue.unwrap_or(Cue::Ready)),
            urgency: Urgency::Interrupt,
        }
    }

    pub fn with_fallback(mut self, cue: Cue) -> Self {
        self.fallback = Some(cue);
        self
    }
}

/// Tone-only prompts older than this are pointless (countdown seconds).
const TONE_MAX_AGE: Duration = Duration::from_millis(350);
/// Spoken prompts older than this describe a situation that has moved on.
const SPEECH_MAX_AGE: Duration = Duration::from_millis(2500);
const QUEUE_CAPACITY: usize = 16;
/// Poll interval while a sentence plays (interrupt and cancel latency).
const SPEAKING_POLL: Duration = Duration::from_millis(40);

#[derive(Clone, Debug)]
struct Queued {
    prompt: Prompt,
    generation: u64,
    queued_at: Instant,
}

/// Pure queue policy: from everything waiting, choose at most one prompt.
/// Prompts from an old generation or older than their maximum age are dropped;
/// the newest interrupting prompt supersedes all older ones; background
/// prompts are dropped while a sentence is playing.
fn select_prompt(
    waiting: Vec<Queued>,
    now: Instant,
    generation: u64,
    speaking: bool,
) -> Option<Queued> {
    let fresh: Vec<Queued> = waiting
        .into_iter()
        .filter(|q| q.generation == generation)
        .filter(|q| {
            let max_age = if q.prompt.text.is_some() {
                SPEECH_MAX_AGE
            } else {
                TONE_MAX_AGE
            };
            now.saturating_duration_since(q.queued_at) <= max_age
        })
        .collect();
    if let Some(latest) = fresh
        .iter()
        .rev()
        .find(|q| q.prompt.urgency == Urgency::Interrupt)
    {
        return Some(latest.clone());
    }
    if speaking {
        return None;
    }
    fresh.into_iter().next_back()
}

/// Text-to-speech output. Implementations must not block for longer than the
/// synthesis of one short sentence.
pub trait SpeechBackend {
    fn speak(&mut self, text: &str) -> Result<(), String>;
    fn stop(&mut self);
    fn is_speaking(&mut self) -> bool;
}

/// Tone output (synchronous).
pub trait TonePlayer {
    fn tone(&mut self, frequency_hz: u32, duration_ms: u32);
    fn pause(&mut self, duration_ms: u32) {
        std::thread::sleep(Duration::from_millis(duration_ms.into()));
    }
}

type BackendFactory = Box<dyn FnOnce() -> Result<Box<dyn SpeechBackend>, String> + Send>;

struct Shared {
    generation: AtomicU64,
    mode: AtomicU8,
    voice_status: Mutex<String>,
}

pub struct AudioCues {
    sender: Option<SyncSender<Queued>>,
    shared: Arc<Shared>,
}

impl AudioCues {
    pub fn new() -> Self {
        Self::with_parts(
            Box::new(platform_voice),
            Box::new(PlatformTones),
            "calibration-audio",
        )
    }

    fn with_parts(voice: BackendFactory, tones: Box<dyn TonePlayer + Send>, name: &str) -> Self {
        let (sender, receiver) = sync_channel::<Queued>(QUEUE_CAPACITY);
        let shared = Arc::new(Shared {
            generation: AtomicU64::new(0),
            mode: AtomicU8::new(AudioMode::default().to_u8()),
            voice_status: Mutex::new("Voice not used yet.".into()),
        });
        let worker_shared = shared.clone();
        let worker = std::thread::Builder::new()
            .name(name.into())
            .spawn(move || audio_worker(receiver, worker_shared, voice, tones));
        Self {
            sender: worker.ok().map(|_| sender),
            shared,
        }
    }

    /// Nonblocking best effort. A slow or unavailable sound device never delays
    /// calibration, network ingestion or the live solver.
    pub fn play(&self, prompt: Prompt) {
        if let Some(sender) = &self.sender {
            let _ = sender.try_send(Queued {
                prompt,
                generation: self.shared.generation.load(Ordering::Relaxed),
                queued_at: Instant::now(),
            });
        }
    }

    pub fn tone(&self, cue: Cue) {
        self.play(Prompt::tone(cue));
    }

    /// Silences everything queued or playing.
    pub fn cancel(&self) {
        self.shared.generation.fetch_add(1, Ordering::Relaxed);
    }

    pub fn set_mode(&self, mode: AudioMode) {
        self.shared.mode.store(mode.to_u8(), Ordering::Relaxed);
    }

    pub fn mode(&self) -> AudioMode {
        AudioMode::from_u8(self.shared.mode.load(Ordering::Relaxed))
    }

    /// Human-readable state of speech synthesis (for the UI).
    pub fn voice_status(&self) -> String {
        self.shared
            .voice_status
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

impl Drop for AudioCues {
    fn drop(&mut self) {
        // Closing the sender lets the worker exit; invalidate anything it has
        // already dequeued without waiting on a sound device during shutdown.
        self.cancel();
    }
}

fn set_status(shared: &Shared, text: impl Into<String>) {
    *shared
        .voice_status
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = text.into();
}

fn audio_worker(
    receiver: Receiver<Queued>,
    shared: Arc<Shared>,
    voice: BackendFactory,
    mut tones: Box<dyn TonePlayer + Send>,
) {
    let mut factory = Some(voice);
    let mut backend: Option<Box<dyn SpeechBackend>> = None;
    // Generation of the sentence currently playing.
    let mut speaking_generation = None;
    loop {
        let speaking =
            speaking_generation.is_some() && backend.as_mut().is_some_and(|b| b.is_speaking());
        if !speaking {
            speaking_generation = None;
        }
        let first = if speaking {
            match receiver.recv_timeout(SPEAKING_POLL) {
                Ok(queued) => Some(queued),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => break,
            }
        } else {
            match receiver.recv() {
                Ok(queued) => Some(queued),
                Err(_) => break,
            }
        };
        let generation = shared.generation.load(Ordering::Relaxed);
        let mode = AudioMode::from_u8(shared.mode.load(Ordering::Relaxed));
        if speaking_generation.is_some_and(|g| g != generation) || mode != AudioMode::VoiceAndTones
        {
            if let Some(backend) = backend.as_mut() {
                backend.stop();
            }
            speaking_generation = None;
        }
        let Some(first) = first else {
            continue;
        };
        let mut waiting = vec![first];
        waiting.extend(receiver.try_iter());
        let speaking = speaking_generation.is_some();
        let Some(chosen) = select_prompt(waiting, Instant::now(), generation, speaking) else {
            continue;
        };
        if mode == AudioMode::Silent {
            continue;
        }
        if speaking && let Some(backend) = backend.as_mut() {
            backend.stop();
            speaking_generation = None;
        }
        let prompt = chosen.prompt;
        let current = || shared.generation.load(Ordering::Relaxed) == chosen.generation;
        if let Some(cue) = prompt.cue {
            play_tones(tones.as_mut(), cue, &current);
        }
        let Some(text) = prompt.text.as_deref() else {
            continue;
        };
        if !current() {
            continue;
        }
        let mut spoken = false;
        if mode == AudioMode::VoiceAndTones {
            if backend.is_none()
                && let Some(create) = factory.take()
            {
                match create() {
                    Ok(created) => {
                        set_status(&shared, "Windows speech synthesis ready.");
                        backend = Some(created);
                    }
                    Err(error) => set_status(
                        &shared,
                        format!("Speech unavailable ({error}); using tones."),
                    ),
                }
            }
            if let Some(voice) = backend.as_mut() {
                match voice.speak(text) {
                    Ok(()) => {
                        spoken = true;
                        speaking_generation = Some(chosen.generation);
                    }
                    Err(error) => {
                        set_status(&shared, format!("Speech failed ({error}); using tones."))
                    }
                }
            }
        }
        if !spoken
            && prompt.cue.is_none()
            && let Some(fallback) = prompt.fallback
        {
            play_tones(tones.as_mut(), fallback, &current);
        }
    }
    if let Some(backend) = backend.as_mut() {
        backend.stop();
    }
}

fn play_tones(player: &mut dyn TonePlayer, cue: Cue, current: &dyn Fn() -> bool) {
    for &(frequency_hz, duration_ms) in tones(cue) {
        if !current() {
            break;
        }
        if frequency_hz == 0 {
            player.pause(duration_ms);
        } else {
            player.tone(frequency_hz, duration_ms);
        }
    }
}

fn tones(cue: Cue) -> &'static [(u32, u32)] {
    match cue {
        Cue::Countdown => &[(880, 70)],
        Cue::Ready => &[(1320, 110)],
        Cue::Move => &[(784, 90), (0, 35), (1046, 90), (0, 35), (1320, 160)],
        Cue::Success => &[(1046, 110), (0, 45), (1568, 150)],
        Cue::Failure => &[(392, 150), (0, 60), (330, 220)],
        Cue::Cancelled => &[(440, 90)],
    }
}

struct PlatformTones;

impl TonePlayer for PlatformTones {
    fn tone(&mut self, frequency_hz: u32, duration_ms: u32) {
        platform_tone(frequency_hz, duration_ms);
    }
}

#[cfg(windows)]
fn platform_tone(frequency_hz: u32, duration_ms: u32) {
    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn Beep(frequency: u32, duration: u32) -> i32;
    }
    // SAFETY: Beep accepts values, retains no memory, and each internal cue uses
    // a frequency in its documented 37..=32767 Hz range with a short duration.
    // See https://learn.microsoft.com/windows/win32/api/utilapiset/nf-utilapiset-beep
    let _ = unsafe { Beep(frequency_hz, duration_ms) };
}

#[cfg(not(windows))]
fn platform_tone(_frequency_hz: u32, _duration_ms: u32) {}

#[cfg(windows)]
fn platform_voice() -> Result<Box<dyn SpeechBackend>, String> {
    winrt_voice::WinRtVoice::new()
        .map(|voice| Box::new(voice) as Box<dyn SpeechBackend>)
        .map_err(|error| error.message().to_string())
}

#[cfg(not(windows))]
fn platform_voice() -> Result<Box<dyn SpeechBackend>, String> {
    Err("speech synthesis is only implemented on Windows".into())
}

/// Windows speech through WinRT, usable from a desktop (non-packaged) process.
/// `SpeechSynthesizer` and `MediaPlayer` are agile; windows-rs joins the
/// multithreaded apartment implicitly on first activation.
/// https://learn.microsoft.com/uwp/api/windows.media.speechsynthesis.speechsynthesizer
/// https://learn.microsoft.com/uwp/api/windows.media.playback.mediaplayer
#[cfg(windows)]
mod winrt_voice {
    use super::SpeechBackend;
    use std::time::{Duration, Instant};
    use windows::{
        Media::{
            Core::MediaSource,
            Playback::{
                IMediaPlaybackSource, MediaPlaybackState, MediaPlayer, MediaPlayerAudioCategory,
            },
            SpeechSynthesis::{SpeechAppendedSilence, SpeechSynthesizer},
        },
        core::{HSTRING, Result},
    };

    /// A prompt longer than this is assumed finished (safety net).
    const MAX_SENTENCE: Duration = Duration::from_secs(8);
    /// Grace before a paused/idle session counts as finished (opening).
    const START_GRACE: Duration = Duration::from_millis(400);

    pub struct WinRtVoice {
        synthesizer: SpeechSynthesizer,
        player: MediaPlayer,
        started: Option<Instant>,
    }

    impl WinRtVoice {
        pub fn new() -> Result<Self> {
            let synthesizer = SpeechSynthesizer::new()?;
            // Fails when no Microsoft-signed voice is installed.
            SpeechSynthesizer::DefaultVoice()?;
            if let Ok(options) = synthesizer.Options() {
                // Optional (Windows 10 1709/1803+): slightly brisk, no trailing pause.
                let _ = options.SetSpeakingRate(1.1);
                let _ = options.SetAppendedSilence(SpeechAppendedSilence::Min);
            }
            let player = MediaPlayer::new()?;
            let _ = player.SetAudioCategory(MediaPlayerAudioCategory::Speech);
            // Keep calibration prompts out of the system media overlay.
            if let Ok(commands) = player.CommandManager() {
                let _ = commands.SetIsEnabled(false);
            }
            Ok(Self {
                synthesizer,
                player,
                started: None,
            })
        }

        fn try_speak(&mut self, text: &str) -> Result<()> {
            let stream = self
                .synthesizer
                .SynthesizeTextToStreamAsync(&HSTRING::from(text))?
                .join()?;
            let source = MediaSource::CreateFromStream(&stream, &stream.ContentType()?)?;
            self.player.SetSource(&source)?;
            self.player.Play()?;
            self.started = Some(Instant::now());
            Ok(())
        }
    }

    impl SpeechBackend for WinRtVoice {
        fn speak(&mut self, text: &str) -> std::result::Result<(), String> {
            self.try_speak(text)
                .map_err(|error| error.message().to_string())
        }

        fn stop(&mut self) {
            let _ = self.player.Pause();
            let _ = self.player.SetSource(None::<&IMediaPlaybackSource>);
            self.started = None;
        }

        fn is_speaking(&mut self) -> bool {
            let Some(started) = self.started else {
                return false;
            };
            if started.elapsed() > MAX_SENTENCE {
                self.stop();
                return false;
            }
            let state = self
                .player
                .PlaybackSession()
                .and_then(|session| session.PlaybackState());
            let active = match state {
                Ok(state) => {
                    state == MediaPlaybackState::Opening
                        || state == MediaPlaybackState::Buffering
                        || state == MediaPlaybackState::Playing
                        || started.elapsed() < START_GRACE
                }
                Err(_) => false,
            };
            if !active {
                self.started = None;
            }
            active
        }
    }

    #[cfg(test)]
    mod tests {
        /// Plays one real sentence on the default audio device. Ignored by
        /// default: it needs installed voices and makes a sound.
        #[test]
        #[ignore]
        fn speaks_a_sentence_on_this_machine() {
            use super::super::SpeechBackend;
            let mut voice = super::WinRtVoice::new().expect("speech synthesis");
            voice.speak("ChadFBT voice test.").expect("speak");
            let start = std::time::Instant::now();
            while voice.is_speaking() && start.elapsed().as_secs() < 8 {
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
            assert!(
                start.elapsed().as_millis() > 300,
                "sentence ended immediately"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    fn queued(prompt: Prompt, generation: u64, queued_at: Instant) -> Queued {
        Queued {
            prompt,
            generation,
            queued_at,
        }
    }

    #[test]
    fn stale_and_superseded_prompts_are_dropped() {
        let base = Instant::now();
        let now = base + Duration::from_millis(3000);
        let old_tone = queued(Prompt::tone(Cue::Countdown), 0, base);
        let old_speech = queued(Prompt::say(None, "old"), 0, base);
        assert!(select_prompt(vec![old_tone, old_speech], now, 0, false).is_none());

        let fresh = base + Duration::from_millis(2900);
        let first = queued(Prompt::say(None, "first"), 0, fresh);
        let second = queued(Prompt::say(None, "second"), 0, fresh);
        let tick = queued(Prompt::tone(Cue::Countdown), 0, fresh);
        let chosen = select_prompt(vec![first, second, tick], now, 0, false).unwrap();
        assert_eq!(chosen.prompt.text.as_deref(), Some("second"));
    }

    #[test]
    fn cancelled_generation_and_background_while_speaking_are_dropped() {
        let now = Instant::now();
        let cancelled = queued(Prompt::say(None, "before cancel"), 0, now);
        assert!(select_prompt(vec![cancelled], now, 1, false).is_none());
        let tick = queued(Prompt::tone(Cue::Countdown), 1, now);
        assert!(select_prompt(vec![tick.clone()], now, 1, true).is_none());
        assert!(select_prompt(vec![tick], now, 1, false).is_some());
    }

    #[derive(Debug, PartialEq)]
    enum Event {
        Tone(u32),
        Speak(String),
        Stop,
    }

    struct FakeVoice {
        events: mpsc::Sender<Event>,
        fail: bool,
        busy_until: Option<Instant>,
    }

    impl SpeechBackend for FakeVoice {
        fn speak(&mut self, text: &str) -> Result<(), String> {
            if self.fail {
                return Err("no voice".into());
            }
            let _ = self.events.send(Event::Speak(text.into()));
            self.busy_until = Some(Instant::now() + Duration::from_millis(400));
            Ok(())
        }
        fn stop(&mut self) {
            if self.busy_until.take().is_some() {
                let _ = self.events.send(Event::Stop);
            }
        }
        fn is_speaking(&mut self) -> bool {
            self.busy_until.is_some_and(|until| Instant::now() < until)
        }
    }

    struct FakeTones(mpsc::Sender<Event>);

    impl TonePlayer for FakeTones {
        fn tone(&mut self, frequency_hz: u32, _duration_ms: u32) {
            let _ = self.0.send(Event::Tone(frequency_hz));
        }
        fn pause(&mut self, _duration_ms: u32) {}
    }

    fn fake(fail: bool) -> (AudioCues, mpsc::Receiver<Event>) {
        let (tx, rx) = mpsc::channel();
        let voice_tx = tx.clone();
        let cues = AudioCues::with_parts(
            Box::new(move || {
                Ok(Box::new(FakeVoice {
                    events: voice_tx,
                    fail,
                    busy_until: None,
                }) as Box<dyn SpeechBackend>)
            }),
            Box::new(FakeTones(tx)),
            "audio-test",
        );
        (cues, rx)
    }

    fn collect(rx: &mpsc::Receiver<Event>, wait: Duration) -> Vec<Event> {
        let mut events = Vec::new();
        while let Ok(event) = rx.recv_timeout(wait) {
            events.push(event);
        }
        events
    }

    #[test]
    fn worker_speaks_after_tones_and_a_new_prompt_interrupts() {
        let (cues, rx) = fake(false);
        cues.play(Prompt::say(Some(Cue::Success), "Calibration complete."));
        let events = collect(&rx, Duration::from_millis(150));
        assert_eq!(
            events,
            vec![
                Event::Tone(1046),
                Event::Tone(1568),
                Event::Speak("Calibration complete.".into())
            ]
        );
        // A countdown tick during speech is dropped; a new sentence interrupts.
        cues.tone(Cue::Countdown);
        cues.play(Prompt::say(None, "Calibration cancelled."));
        let events = collect(&rx, Duration::from_millis(150));
        assert_eq!(
            events,
            vec![Event::Stop, Event::Speak("Calibration cancelled.".into())]
        );
        cues.cancel();
        let events = collect(&rx, Duration::from_millis(150));
        assert_eq!(events, vec![Event::Stop]);
    }

    #[test]
    fn tones_only_silent_and_unavailable_voice_fall_back() {
        let (cues, rx) = fake(true);
        cues.play(Prompt::say(None, "Heading reset.").with_fallback(Cue::Success));
        let events = collect(&rx, Duration::from_millis(150));
        assert_eq!(events, vec![Event::Tone(1046), Event::Tone(1568)]);
        assert!(
            cues.voice_status().contains("no voice"),
            "{}",
            cues.voice_status()
        );

        let (cues, rx) = fake(false);
        cues.set_mode(AudioMode::TonesOnly);
        cues.play(Prompt::say(Some(Cue::Cancelled), "Calibration cancelled."));
        assert_eq!(
            collect(&rx, Duration::from_millis(150)),
            vec![Event::Tone(440)]
        );
        cues.set_mode(AudioMode::Silent);
        cues.play(Prompt::say(
            Some(Cue::Failure),
            "Calibration did not finish.",
        ));
        cues.tone(Cue::Countdown);
        assert!(collect(&rx, Duration::from_millis(150)).is_empty());
        assert_eq!(cues.mode(), AudioMode::Silent);
    }
}
