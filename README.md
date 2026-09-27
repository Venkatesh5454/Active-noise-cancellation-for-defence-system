# AI/ML-enabled Adaptive Noise Cancellation for Defence Communication
**Smart India Hackathon 2026 · Problem Statement 26052 (DRDO)**

MATLAB implementation of a hybrid (DSP + AI/ML) adaptive noise canceller that
suppresses **stationary** (engine / rotor hum, wind), **non-stationary**
(missile & turbine whine, helicopter, drones, sirens) and **impulsive**
(gunshots, machine-gun fire, artillery) noise while keeping speech
intelligible. It runs causally in 8 ms blocks with 32 ms latency and needs **no MATLAB
toolboxes**.

## Quick start

1. Open this folder in MATLAB.
2. Run `main_defence_anc` (or the single-file version `defence_anc_all_in_one`).

The script:

* cleans the recordings in `audio/noisy/` and writes `results/<name>_enhanced.wav`,
* **plays** the noisy and the enhanced audio and plots both spectrograms,
* prints an estimated SNR for your recordings,
* measures **SNR, STOI and PESQ** (wide-band P.862.2 and narrow-band P.862.1)
  on known clean speech mixed with defence noise.

To clean any other file:

```matlab
addpath(genpath('src'));
[x, fs] = read_audio('my_noisy_file.mp3');   % wav / mp3 / mpeg / flac ...
p = anc_params();  p.model = dnn_load();     % AI/ML model (models/defence_gru.mat)
[y, fs_out] = defence_anc(x, fs, p);          % y = enhanced speech @ 16 kHz
sound(y, fs_out);  audiowrite('clean.wav', y, fs_out);
```

If you have the clean speech that is inside a noisy file, set
`clean_reference` in `main_defence_anc.m` to get the exact SNR / STOI / PESQ of
that file.

## How it works

```
 mic(s) ─► resample 16 kHz ─► DC block ─► [NLMS adaptive canceller] ─► STFT 32 ms / 8 ms hop
                                            (only with a 2nd,                  │
                                             reference microphone)             ▼
                              SPP-MMSE stationary-noise tracker ─► λ_s(k)   log|Y(k)|²
                                                                   └──────┬──────┘
                                         GRU mask estimator (AI/ML, causal, 0.5 M params)
                                                                          │ gain G(k)
                                       zero-phase low-cut (rumble) ─► G·Y ─► ISTFT ─► speech
```

| Stage | What it does | File |
|---|---|---|
| Reference-mic ANC | NLMS (Widrow) canceller for a primary + reference microphone headset; adaptation is frozen while speech is present | `src/anc/nlms_anc.m` |
| Noise tracker | SPP-based MMSE noise-PSD tracker (Gerkmann & Hendriks 2012) for stationary noise | `src/anc/anc_process_frame.m` |
| **AI/ML mask estimator** | dense → GRU → dense network. Its input is the noisy log-spectrum plus the tracked noise PSD ("noise-aware"); its output is a per-bin gain. It is trained on speech mixed with gunshot, machine-gun, artillery, helicopter, missile, drone, siren, vehicle, hum and wind noise | `src/anc/dnn_mask_step.m`, `models/defence_gru.mat` |
| Statistical fallback | Impulsive-noise and tonal (whine) estimators with an OM-LSA optimal gain, used when no model file is present | `src/anc/anc_process_frame.m` |
| Real-time engine | `anc_process_frame` processes one 8 ms block. `anc_realtime_demo.m` runs it live between microphone and headphones (Audio Toolbox), or block by block on a file | `anc_realtime_demo.m` |

## Metrics

STOI, PESQ and SNR are *intrusive* metrics: they compare the output with the
**clean** speech. A real battlefield recording has no clean version, so:

* for your recordings, the script prints a **blind SNR estimate**. Speech and
  pause frames are found with a VAD, and the pause frames give the noise level;
* for the official numbers, the script mixes known clean speech with defence
  noise at a known SNR, cleans it, and computes SNR / STOI / PESQ against the
  clean original. This follows the evaluation protocol in the problem
  statement (a dataset of noisy/clean pairs).

The metric code is toolbox-free and validated:

* `stoi_score.m` (STOI and extended STOI) matches `pystoi` to within 5·10⁻⁷.
* `pesq_score.m` (ITU-T P.862 / P.862.1 / P.862.2, independent MATLAB
  re-implementation) matches the ITU-T reference C code to within **0.0001
  MOS** on 58 test cases. These cover noise types, SNRs, fixed and variable
  delay, 8 and 16 kHz, NB and WB, and long files.



## Repository layout

```
main_defence_anc.m          one-click demo: clean, play, plot, SNR / STOI / PESQ
defence_anc_all_in_one.m    the same program bundled into a single file
anc_realtime_demo.m         live microphone -> headphone streaming demo
models/                     trained network (MATLAB .mat + ONNX for Jetson/TensorRT)
audio/noisy/                sample defence recordings
audio/clean/                clean test speech (CMU ARCTIC) for the objective tests
src/anc/                    noise canceller (streaming engine, GRU inference, NLMS)
src/metrics/                STOI, PESQ, SNR / SegSNR / SI-SDR, blind SNR
src/dataset/                defence-noise synthesiser, SNR mixing, dataset generator
src/utils/                  resampler, audio reader, plots
tools/train_dnn.py          PyTorch training script for the mask estimator
tools/make_single_file.py   rebuilds defence_anc_all_in_one.m
jetson/                     real-time runner for NVIDIA Jetson (Python, same algorithm)
```

## Training / scaling up

```matlab
make_noise_bank('noisebank', 60, 12);           % 11 defence noise types x 60 clips
make_dataset('audio/clean', 'dataset', 1000);   % noisy/clean pairs + manifest.csv
```
```bash
pip install torch soundfile scipy onnx
python tools/train_dnn.py --speech <clean_speech_dir> --noise noisebank <other_noise_dirs> --out models
```

The shipped model was trained on MS-SNSD clean speech (Edinburgh 56-speaker /
VCTK and PTDB-TUG, 58 speakers, 3.8 h), the synthetic defence noise bank
(2.2 h), and MS-SNSD noise (3.9 h). SNRs were drawn from −10 to +20 dB, with
random level, EQ and reverberation. `models/defence_gru.onnx` is the same
network exported for ONNX Runtime / TensorRT, and `models/defence_gru.npz`
holds the same weights for NumPy.

## Deploying on NVIDIA Jetson

`jetson/anc_jetson.py` runs the same algorithm in real time on a Jetson (or
any Linux PC): microphone → noise canceller → headphones, in 8 ms blocks.
Its output matches the MATLAB output to < 3·10⁻⁶. Step-by-step setup (JetPack,
audio devices, live demo, TensorRT, autostart) is in
[`jetson/README.md`](jetson/README.md).

```bash
pip3 install numpy soundfile sounddevice onnxruntime
python3 jetson/anc_jetson.py --file audio/noisy/defence_sound.mp3 --out cleaned.wav
python3 jetson/anc_jetson.py --live --in-dev 11 --out-dev 11     # see --list-devices
```

## Credits and licences

* CMU ARCTIC speech (`audio/clean`): see `audio/clean/LICENSE_CMU_ARCTIC.txt`.
* Training data: MS-SNSD (Microsoft, MIT), containing the Edinburgh 56-speaker
  dataset (CC BY 4.0), PTDB-TUG (ODbL), Freesound CC0 and DEMAND (CC BY-SA 3.0).
* PESQ is the intellectual property of OPTICOM GmbH and Psytechnics Ltd.
  `pesq_score.m` is an independent re-implementation for research and
  educational use. Commercial use requires a licence from the IPR owners.
