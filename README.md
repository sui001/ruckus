# Ruckus

Hobby servos' own mechanical noise as a sound source. It started as a
note-scan test on single servos (below) and grew into the motor player rig:
eight continuous servos, tuned by ear with a mic, playing tunes. The repo
was called Servo-notes until 19 Oct 2026.

## Servo + INMP441 note-scan test

## Wiring
- INMP441: VDD→3V3, GND→GND, L/R→GND (left channel), WS→GPIO12, SCK→GPIO11, SD→GPIO13
- Servo signal→GPIO8. For a 9g SG92R the board's own 5V pin is fine; share
  ground with the ESP32. Avoid strapping pins 0/3/45/46.

## Flashing
1. Arduino IDE, board = "ESP32S3 Dev Module" (or the Supermini variant if listed).
2. Install `ESP32Servo` library.
3. In `servo_mic_test.ino`, set `TEST_POSITIONAL` / `TEST_CONTINUOUS` for
   whichever unit is wired up right now — only one at a time, since they're
   physically different servos.
4. Flash, then run `analyze.py <port> <name>` on the computer — it waits
   for the plan, then records the whole run and saves `<name>.wav` +
   `<name>.csv`.

## What's tested
**Positional (180°) servo:** 9 target distances (5°–150° from home) ×
4 command-rate profiles (jump, and 20/10/5 ms per degree) × 3 repeats
= 108 events.

**Continuous-rotation servo:** 7 speed levels × both directions × 3
repeats = 42 events.

Note: a 180° servo can't be turned into continuous-rotation by code —
that needs the actual continuous-rotation part (different gearing, no
end-stop). So "test both" means having one of each in hand, run as two
separate flashes.

## Output
`analyze.py` gives a CSV of every event's detected frequency, nearest
equal-tempered note (A4=440), and cents off. Low `confidence` rows mean
the segment was noise-like rather than a clear pitch — expected for some
combos, and itself useful information (that region isn't a usable "note").

## What the recordings found (14 Sep 2026, SG92R)

Measured, on clean unclipped audio:

- The noise peaks at **~9 kHz and stays there**, whatever speed or distance is
  commanded (jump 8648 Hz, 5 ms/deg 8914, 10 ms/deg 8953, 20 ms/deg 8906). It is
  a fixed property of the mechanism.
- **Movement speed does not set pitch.** That was the idea the project started
  from, and it is refuted rather than merely unsupported: the fastest setting
  gave the lowest peak, not the highest.
- It is a **noise band, not a tone**: spectral flatness ~0.45, no harmonic
  structure. This is a percussion and texture instrument, not a melodic one, so
  "eight notes and chords" does not apply and there is no per-servo calibration
  table worth building.
- **Loudness does track the command.** Move RMS ran 44 / 79 / 148 / 246 for
  1 / 2 / 3 / 5 degree steps. Step size is a velocity control.
- Still open: whether each commanded step makes its own tick. Three measurement
  methods disagreed and most events failed the onset detector's own regularity
  test, so it is recorded as unresolved rather than answered.

`servo_pattern.ino` is what follows from this: a hit, a velocity and a rhythm,
with no note parameter, because there is no note.

## Up vs down (servo_direction_test, 14 Sep 2026)

Found by ear first. The sequencer alternates direction on every hit, and it
sounded like a high and a low drum. None of the earlier tests could have caught
this: they measured only outbound moves and discarded every return as gap
contamination.

Isolated single moves, 8 repeats each way, measured on the move burst only.
`d` is the up/down difference divided by the spread across repeats, so above
about 1 means the difference is bigger than the servo's own inconsistency.

| move | centroid up / down | d | loudness up / down | d |
|---|---|---|---|---|
| 2° | 6844 / 5580 Hz | +0.7 | 161 / 152 | +0.5 |
| 4° | 7249 / 6918 Hz | +0.6 | 156 / 167 | -0.6 |
| 6° | 9502 / 8606 Hz | **+5.0** | 374 / 386 | -0.3 |
| 8° | 9767 / 9013 Hz | **+7.0** | 607 / 649 | -2.6 |
| 12° | 9087 / 8718 Hz | +1.8 | 835 / 1016 | **-3.4** |

- **From 6° up, the two directions are clearly two sounds.** Up is brighter,
  about 900 Hz higher in centroid and 2 dB stronger above 6 kHz. Down is the
  duller, lower one, and at 12° also the louder one. At 12° the down move has a
  resonance near 6 kHz that the up move lacks.
- **At 2-4° there is no reliable difference.** Small moves vary more from one
  repeat to the next than up differs from down.
- The sequencer's `V 8` moves 8° per hit, so it was already sitting in the range
  where the effect is strongest. That is why it was audible.
- This is a timbre difference, hi/lo like two toms, not a pitch interval: the
  ~9 kHz peak is the same both ways.
- Caveat: an up move starts at centre and a down move at centre plus the move,
  so direction and start position are measured together here. Over a few
  degrees that is unlikely to matter, but it is not ruled out.

The CSV for this run is not the measurement above. `analyze.py` skipped the
first 100 ms of every window, and these moves peak at 40 ms, so `dirtest.csv`
describes the servo holding still (flatness 0.08, low lines between 47 and
300 Hz that look like room and mains hum rather than servo). The move itself
has flatness 0.55, a noise band like every other recording. Sketches now declare
`skip_ms` in their plan, and this one declares 0.

## Continuous-rotation servo (14 Sep 2026)

On this part `writeMicroseconds` sets actual motor speed, and a peak below
2 kHz follows it. It is the only place in the project a servo has shown pitch
that can be steered.

- Range, measured before each tune by sweeping 100-500 us from stop in 10 us
  steps, turning one way: 888-1379 Hz on the last run, about 7 semitones.
  Below about 140 us the servo is in its deadband, barely turning, and the
  loudest bin is noise.
- **It will not hold a note precisely.** Three attempts at the first phrase of
  Happy Birthday put 3, 1 and 1 of 6 notes within 30 cents. Two things go wrong:
  - The same speed command does not reliably give the same pitch. In the second
    and third runs the G played 924 and 928 Hz at the start of the phrase, and
    1038 and 1037 Hz later in it. Approaching that G from below in the third
    run did not move it, so it is not simply which way the motor comes to the
    speed.
  - Around 1100-1650 Hz the strongest peak hops between partials within one
    held note (1110, 1265 and 1615 Hz inside one B), so the note wavers and a
    peak-picker cannot say what it is.
- So: steerable pitch, not tuned pitch. Holding a note to within 30 cents would
  need the mic in the loop, correcting the speed while the note plays.

`conttest.wav` is the speed sweep that found the effect. `tune.wav`,
`tune2.wav` and `tune3.wav` are the three attempts; the second and third have
a `.cal.json` with the calibration sweep each one used.

## Recordings

Kept as evidence, including the ones that went wrong.

| file | what it is |
|---|---|
| `firsttest.wav` | First real capture. 16 kHz, mic ~5 cm. Usable but band-limited: servo energy was still climbing at the 8 kHz Nyquist edge, so the spectrum was cut off rather than captured. Its CSV is worthless, see below. |
| `secondtest.wav` | 32 kHz, mic moved to 1 cm. **Clipped**, 1.09% of samples at full scale across 50 of 108 events, because the firmware converted the mic at `>>14` instead of `>>16`. Clipping invents broadband harmonics, which corrupts exactly the tonal-or-noise question the rig exists to answer. Kept because the failure is instructive. |
| `thirdtest.wav` | 32 kHz, 1 cm, gain corrected. Clean, peak 7432/32767. **This is the one the findings above come from.** |
| `steptest.wav` | First step-size sweep. Clipped at the loudest step size; timing still usable, spectrum not. |
| `steptest2.wav` | Step-size sweep re-run at `>>17`. Clean, peak 2694/32767. |
| `dirtest.wav` | Up vs down, isolated 2-12° moves at `>>14`. Clean, peak 8367/32767. Findings are in the Up vs down section above; its CSV measured the servo holding still, not the moves. |

Each `.wav` has a `.csv` of its per-event measurements, and every recording from
`secondtest` on has a `.plan.json` beside it so the analysis can be re-run
without re-recording. `firsttest` predates that, and its plan would have to be
reconstructed from the firmware as it stood at the time.

The `firsttest.csv` numbers should not be trusted: the detector searched
60-4000 Hz, below where the noise actually is, and returned an autocorrelation
argmax that on broadband noise always lands on the lag beside zero. All 108
events reported one of three frequencies, every one sitting on the search
window's floor, at a confidence high enough to look like a real reading. The
detector was rewritten and checked against known tones afterwards.

## Time estimate
- Per positional unit: 108 events × ~1.6s ≈ 3 min recording.
- Per continuous unit: 42 events × ~1.9s ≈ 1.5 min recording.
- If you test 9g positional + one bigger positional + 1–2 continuous
  units: ~4 flashes/swaps, ~10 min of actual recording total.
- Add setup (quiet room, mic placement, re-flashing per unit, checking
  each WAV isn't clipping or full of mains hum): realistically
  **45–60 minutes** for the first full pass across 3–4 servos.

Second pass will be faster once the rig and plan are known-good.

## Motor player rig

`motor_player_rig/` is where this went next: eight continuous servos on a
PCA9685 (ch0-7), an INMP441 mic, and an ESP32-S3 SuperMini. Each motor is
tuned by ear at boot, then the rig plays tunes. The sketches came across from
another repo on 19 Oct 2026, which is why they keep their `gen3d_` names.

| Sketch | What it does |
|---|---|
| `gen3d_mic_test` | INMP441 bring-up: level, pitch and note name over serial |
| `gen3d_pca9685_servo_test` | PCA9685 bring-up: one continuous and one 180 degree servo |
| `gen3d_motor_sound_map` | Runs each servo through its speed range and records what the mic hears |
| `gen3d_pitch_servo` | Sound to servos: pitch, note and level on three positional servos |
| `gen3d_motor_player` | The player: mic-tuned continuous servos as instruments |
