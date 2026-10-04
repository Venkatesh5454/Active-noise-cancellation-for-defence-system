# Training data for the defence noise canceller
### What the AI filter learned from, where each sound came from, how we made it, and how to reuse it

**Smart India Hackathon 2026 · Problem Statement 26052 · DRDO / Department of Defence Production**

The noise canceller in this project contains a small neural network (the GRU
"mask estimator" in `models/defence_gru.*`). A network only knows what it has
heard during training. This guide shows **exactly** what that was:

* clean speech, so the network learns what a voice looks like;
* defence noise (gunshots, machine guns, artillery, helicopters, missiles and
  more), so it learns what to remove;
* everyday noise (traffic, crowds, machines), so it does not mistake every
  other sound for speech.

Everything is packed in the zip files `defence_anc_training_data_*.zip`. In the
GitHub repository, the `dataset/` folder holds this guide, the licence notes,
the file lists (`manifests/`) and the helper scripts, but no audio.

---

## Contents

1. [What is in the package](#1-what-is-in-the-package)
2. [Getting started](#2-getting-started)
3. [The three ingredients and how we made them](#3-the-three-ingredients-and-how-we-made-them)
4. [From ingredients to training examples](#4-from-ingredients-to-training-examples)
5. [The example pairs](#5-the-example-pairs)
6. [Using the data for future work](#6-using-the-data-for-future-work)
7. [Keeping the evaluation fair](#7-keeping-the-evaluation-fair)
8. [Limitations, honestly](#8-limitations-honestly)
9. [Licences and credits](#9-licences-and-credits)
10. [Reference: folders and manifest columns](#10-reference-folders-and-manifest-columns)

---

## 1. What is in the package

| Folder | What it contains | Files | Duration | Where it came from |
|---|---|---|---|---|
| `speech_clean/` | Clean read English sentences, 58 speakers | 4,639 | 3.79 h | Edinburgh noisy-speech database (VCTK speakers), via Microsoft MS-SNSD |
| `noise_defence_synthetic/` | Gunshots, machine-gun bursts, artillery, helicopter, missile, drone, siren, military vehicle, generator hum, wind and a battlefield mix | 660 | 2.20 h | Made by our MATLAB generator `defence_noise.m` |
| `noise_general/` | Everyday noise: babble, traffic, bus, car, metro, café, office, typing, air-conditioner, vacuum cleaner and more | 179 | 3.95 h | Microsoft MS-SNSD (Freesound CC0 and DEMAND recordings) |
| `example_pairs/` | 160 ready-made noisy/clean training examples, 3 s each | 2 × 160 | 2 × 8 min | Made from the three folders above with `make_training_pairs.py` |
| `manifests/` | One row per audio file: duration, level, source, licence, checksums | | | |
| `scripts/` | Everything needed to rebuild, extend and retrain | | | |

**Audio format:** 16 kHz, mono, 16-bit, stored as **FLAC**. FLAC is *lossless*
compression, like a zip file for audio: the samples are exactly the ones
the network was trained on (we checked every file sample by sample), the
files are just 30–50 % smaller. MATLAB, GNU Octave, Python (`soundfile`),
Audacity and VLC all open FLAC directly. If a tool really needs WAV, use
`scripts/flac_to_wav.m` or `scripts/flac_to_wav.py`.

**Total:** 5,478 source files, 9.9 hours of audio, 668 MB.

---

## 2. Getting started

**1. Unzip all parts into the same folder.** The package comes as 25 zip
files, because each download had to stay under 30 MB. Every one is a normal,
complete zip file (not a split archive), so any unzip tool works and the
order does not matter. They all contain the same top folder,
`defence_anc_training_data/`, so the parts simply merge:

| Zip files | Contains | Size |
|---|---|---|
| `defence_anc_training_data_01_of_25_docs_scripts_examples.zip` | this guide (also as PDF), licences, manifests, scripts, example pairs | 16&nbsp;MB |
| `..._02_of_25_speech_clean.zip` to `..._09_of_25_speech_clean.zip` (8 files) | `speech_clean/` | 220&nbsp;MB |
| `..._10_of_25_noise_defence_synthetic.zip` to `..._15_of_25_noise_defence_synthetic.zip` (6 files) | `noise_defence_synthetic/` | 173&nbsp;MB |
| `..._16_of_25_noise_general.zip` to `..._25_of_25_noise_general.zip` (10 files) | `noise_general/` | 276&nbsp;MB |

On Windows: select all 25 zips, right-click, "Extract All" each one into the
same folder (or use 7-Zip: select them all, then *7-Zip → Extract Here*).
On Linux or macOS: `for z in defence_anc_training_data_*.zip; do unzip -o "$z"; done`.

Zip 01 on its own is enough to read the documentation, listen to the
examples, and rebuild everything else (see [Section 6](#6-using-the-data-for-future-work)).

**2. Check that nothing was damaged** during download (optional, needs Python):

```bash
cd defence_anc_training_data
python scripts/verify_dataset.py
```

It compares every file with `manifests/checksums_sha256.txt`, reports each
folder, and names any zip file that is missing or damaged, so you know
exactly which one to download again. On Linux,
`sha256sum -c manifests/checksums_sha256.txt` checks the files too.

**3. Listen to something.** In MATLAB:

```matlab
cd defence_anc_training_data
[x, fs] = audioread('example_pairs/noisy/pair_00150.flac');  sound(x, fs);
[s, fs] = audioread('example_pairs/clean/pair_00150.flac');  sound(s, fs);
```

---

## 3. The three ingredients and how we made them

```
 MS-SNSD clean speech  58 speakers x <=80 sentences -> speech_clean/            3.8 h
 our MATLAB generator  11 types x 60 clips x 12 s   -> noise_defence_synthetic/ 2.2 h
 MS-SNSD noise         all 179 recordings           -> noise_general/           3.9 h
                                   |
        train_dnn.py / make_training_pairs.py: random 3 s mixes,
        SNR -10 ... +20 dB, room echo, microphone colouring, volume
                                   v
               16,128 noisy / clean training examples (13.4 h)
```

### 3.1 Clean speech (`speech_clean/`)

**What it is:** short English sentences read aloud in a quiet studio by 58
speakers with a variety of British-English accents. The file name tells
you the speaker and the sentence: `p234_018.flac` is speaker `p234`,
sentence `018`.

**Where it came from:** Microsoft's open **MS-SNSD** project (Reddy et al.,
Interspeech 2019) offers a ready-made speech-enhancement toolkit on GitHub.
Its `clean_train` folder contains 23,075 clean sentences at 16 kHz. They
come from the University of Edinburgh's *Noisy speech database for training
speech enhancement algorithms and TTS models* (Valentini-Botinhao, 2017),
whose speakers are from the VCTK / Voice Bank corpus.

**How we picked our subset:** using all 23,075 files would have made the
download and training slower without much benefit for a proof of concept.
We therefore:

1. listed all files in MS-SNSD `clean_train` and grouped them by speaker
   (58 speakers);
2. took **up to 80 random sentences from every speaker** (Python
   `random.seed(1)`), so that every voice counts equally: 4,640 files;
3. downloaded them from GitHub. Three downloads failed (`p263_274`,
   `p339_074`, `p339_401`), and two files from a first test download
   (`p234_001`, `p234_002`) stayed in the folder. That gives the
   **4,639 files** the model was actually trained on.

`manifests/speech_clean.csv` lists every file with its original MS-SNSD
path, duration and loudness. Sentences are 1.0 to 16.3 s long (median
2.7 s). We did not edit the audio. The training script only removes any DC
offset and scales each file to the same loudness when it loads it.

### 3.2 Synthetic defence noise (`noise_defence_synthetic/`)

**Why synthetic?** There is no public, freely licensed collection of
gunfire, missile and military-vehicle sounds that we could legally train
on and share. So we wrote a **sound generator** in MATLAB
(`defence_noise.m`) that builds each noise from simple physical ingredients:
short noise bursts, decaying envelopes, harmonics, filters and echoes. We
tuned it by listening to and measuring the two recordings supplied with the
problem statement. For example, the machine-gun rounds in those recordings
last about 50–90 ms, come 9–12 times per second, and the gaps between
rounds are only about 10 dB quieter than the rounds themselves.

**How each sound is made:**

| Type | Class | How the generator builds it |
|---|---|---|
| `gunshot` | impulsive | Single rifle shots at random moments (on average 0.9 per second, at least 0.35 s apart). Each shot: a supersonic "crack" of about 1 ms, a muzzle blast that dies away in 6–12 ms, a low body (900 Hz low-pass, 30–50 ms), a reverberant tail (100–180 ms) and one echo 40–120 ms later. Loudness varies by ±3 dB. |
| `machinegun` | impulsive | Bursts of 1–3 s separated by 0.3–1 s pauses, 9–12 rounds per second with slight timing jitter. Each round: a blast plus a 45–80 ms low body (1.2 kHz low-pass). A reverberant floor fills the gaps between rounds, about 10 dB below the bursts. |
| `artillery` | impulsive | Explosions at random moments (on average one every 3 s, at least 1.2 s apart): a deep boom (250–400 Hz low-pass, dying away over 0.3–0.8 s), a sharp crack and a long rumble, over a low background roar. |
| `helicopter` | non-stationary | Main rotor at 16–22 blade passes per second with impulsive "blade slap" and 10 harmonics, tail rotor at 5.3 times that rate, broadband rotor wash, a faint 5.2–6 kHz turbine whine, and hiss that pulses with the blades. |
| `missile` | non-stationary | A whine that rises from 550–800 Hz to 1.8–2.6 times that pitch over the clip (with harmonics and a slight vibrato), plus a rocket roar that grows louder. |
| `drone` | non-stationary | Four rotors at 170–210 Hz, each with 6 harmonics and a slow speed wobble, plus soft broadband noise. |
| `siren` | non-stationary | Either a "wail" sweeping 650→1400→650 Hz every 3.5–4.5 s (60 % of clips) or a fast "yelp" (0.25–0.35 s sweeps). |
| `vehicle` | stationary | Armoured-vehicle diesel engine firing 32–44 times per second (12 harmonics), low rumble, track-link clatter 12–18 times per second, and 100 Hz alternator hum. |
| `hum` | stationary | 100 Hz generator hum with 8 harmonics. |
| `wind` | stationary | Deep rumbling noise (below 500 Hz) with slow random gusts of up to three times the level. |
| `battlefield` | mixture | Machine-gun bursts + rifle shots + missile whine + vehicle engine, mixed together like the supplied recordings. |

**How the bank was made:** one MATLAB command,

```matlab
make_noise_bank('noise_defence_synthetic', 60, 12)   % 11 types x 60 clips x 12 s
```

which, for clip `k` of type number `i` (in the order of the table above,
`gunshot` = 1 ... `battlefield` = 11), runs

```matlab
n = defence_noise(type, 12, 16000, 100000 + 1000*i + k);   % seed = 100000 + 1000*i + k
n = 0.99 * n / max(abs(n));                                % peak at 99 % of full scale
```

and saves the result as `gunshot_001.flac`, `gunshot_002.flac` and so on
(`make_noise_bank` writes `.wav`; we converted to FLAC). Every row of
`manifests/noise_defence_synthetic.csv` repeats the exact command for that
file. Because every clip has its own seed, every clip is different, but the
same seed always gives the same clip:

* in **GNU Octave 8.4** (which we used) the command recreates the files
  **bit for bit**; we checked this;
* **MATLAB** uses a different random-number algorithm, so it creates clips
  that are statistically the same kind of noise but not identical samples.

### 3.3 Everyday noise (`noise_general/`)

**What it is:** all **179** noise recordings of MS-SNSD (128 from its
`noise_train` folder and 51 from `noise_test`). We used both, because we
evaluate on our own defence test set, not on MS-SNSD's.

| Kind of noise | Categories (files) |
|---|---|
| Other people talking | Babble (20), NeighborSpeaking (14), Neighbor (8), AirportAnnouncements (14 + 1) |
| Machines and household | AirConditioner (20), CopyMachine (11), Typing (12), VacuumCleaner (10), WasherDryer (7), Munching (17), ShuttingDoor (18), SqueakyChair (11) |
| Places and transport (5 min each) | Bus, Car, Metro, Traffic, Cafe, CafeTeria, Restaurant, Square, Station, Office, Hallway, Kitchen, LivingRoom, Washing, Field, Park (1 each) |

**Why it is in the mix:** without it, the network would learn "everything
that is not a gunshot or an engine must be speech". The everyday noise
teaches it that a café, a bus or a keyboard is noise too. The babble and
announcement files contain other people's voices, so the network also
learns to push *background* talkers down.

**Where it came from:** MS-SNSD collected these sounds from **Freesound**
(only files released under CC0) and from the **DEMAND** database of
environmental noise (CC BY-SA 3.0). MS-SNSD does not say which file came
from where. The 16 place and transport recordings are five minutes long and
are named after DEMAND's recording environments, so they are almost
certainly from DEMAND. The file names keep MS-SNSD's split and category:
`noise_train_Babble_3.flac` was `noise_train/Babble_3.wav` in MS-SNSD.

---

## 4. From ingredients to training examples

The network never trained on a fixed list of files. The training script
(`train_dnn.py`) **mixes a fresh example every time it needs one**, so it
practically never hears the same combination twice. One example is made
like this:

1. **Speech:** take a random 3-second stretch of clean speech. Short
   sentences are joined with natural pauses of 0.1–0.5 s.
2. **Room echo** (15 % of examples): add the reverberation of a virtual
   room (echo time 0.15–0.6 s).
3. **Microphone colouring** (30 %): tilt the speech spectrum slightly
   brighter or darker, like a different microphone.
4. **Noise-only** (5 %): remove the speech completely, so the network also
   learns to stay quiet when nobody talks.
5. **Noise:** take a random 3 s stretch of noise, **70 % from the defence
   folder and 30 % from the everyday folder**. In 35 % of examples, layer a
   second noise on top at 20–100 % strength (for example a helicopter plus
   babble). In 30 %, tilt the noise spectrum too.
6. **Loudness ratio:** set the speech-to-noise ratio to a random value
   between **−10 dB** (noise much louder than the voice) and **+20 dB**
   (noise in the background). 5 % of examples get +40 dB (practically
   clean), so the network learns to leave clean speech alone.
7. **Volume:** scale the whole mix to a random level over a 32 dB range,
   from quiet to loud.

The pair (*noisy mix*, *clean speech*) is one training example: the noisy
mix goes into the network, and the clean speech is the answer it should
produce.

**How much the shipped model saw:** training ran with random seed 0 and 16
examples per step. The first 8 × 16 = 128 examples were used only to
measure the average input level (normalisation statistics). The model in
`models/` is the checkpoint after **1,000 training steps**, so it learned
from 16,000 examples. Together with the first 128, the run used
**16,128 examples, 13.4 hours of mixtures**. Training took about ten
minutes on a 4-core CPU.

**You can regenerate that exact training set.** `make_training_pairs.py`
follows the same recipe and draws its random numbers in the same order as
the training script. With seed 0, pair *i* is exactly the *i*-th example of
the training run. We verified this sample by sample (difference 0.0) for
the first 160 examples:

```bash
cd defence_anc_training_data
python scripts/make_training_pairs.py --speech speech_clean \
       --noise noise_defence_synthetic noise_general \
       --out training_pairs --n 16128 --seed 0
```

This writes all 16,128 pairs (about 1.5 GB of FLAC; roughly 5–10 minutes
on an ordinary 4-core PC) plus `training_pairs/pairs.csv`, which records
every random choice. Use
another `--seed` for a different set made with the same recipe.

---

## 5. The example pairs

`example_pairs/` contains the **first 160 examples of the training run**
(the command above with `--n 160`), so you can listen to exactly what the
network was fed:

* `example_pairs/clean/pair_00001.flac`: the target (clean speech);
* `example_pairs/noisy/pair_00001.flac`: the input (speech + noise);
* `example_pairs/pairs.csv`: how each pair was made.

| Column | Meaning |
|---|---|
| `pair` | file name (same name in `clean/` and `noisy/`) |
| `speech` | speech file(s) used: `p316_171.flac@1552` means "3 s starting at sample 1552 of that file"; `name@0+6482` means "the file from its start, placed at sample 6482 of the pair" (short sentences are strung together this way) |
| `reverb_t60_s` | echo time of the virtual room in seconds (0 = no echo) |
| `speech_tilt`, `noise_tilt` | spectral tilt applied (0 = none; −0.5 ... +0.5) |
| `noise_only` | 1 if the speech was removed |
| `noise1`, `noise2`, `noise2_gain` | noise file(s), position and strength of the second noise |
| `snr_db` | speech-to-noise ratio in dB |
| `level_db` | loudness of the mix relative to the reference level, in dB |
| `file_gain` | if a mix would clip in a 16-bit file, both clean and noisy are turned down by this factor (the SNR stays the same) |
| `used_in_training` | pairs 1–128 were used for the normalisation statistics, pairs 129–160 in training steps 1 and 2 |

---

## 6. Using the data for future work

### In MATLAB

The project's functions are in the `src/` folder of the GitHub repository
(`scripts/` here contains copies of the dataset ones).

```matlab
addpath(genpath('<repository>/src'));          % noise canceller, metrics, generator

% clean one example pair with the trained filter and measure it
[s, fs] = audioread('example_pairs/clean/pair_00150.flac');
[x, ~]  = audioread('example_pairs/noisy/pair_00150.flac');
p = anc_params();  p.model = dnn_load();
y = defence_anc(x, fs, p);
% SNR, STOI, PESQ-WB, PESQ-NB of the noisy input (row 1) and the output (row 2)
r = [evaluate_pair(s, x, fs); evaluate_pair(s, y, fs)]

% make your own mixture at 0 dB SNR
[s, fs] = audioread('speech_clean/p234_018.flac');
[n, ~]  = audioread('noise_defence_synthetic/machinegun_001.flac');
x = mix_at_snr(s, n, 0);

% make brand-new defence noise: 5 s of helicopter, seed 42
n = defence_noise('helicopter', 5, 16000, 42);
```

The example pairs were part of the training data, so scores measured on
them look better than on unseen data. Use them for demos. For honest
numbers, use PART 2 of `main_defence_anc.m`
(see [Section 7](#7-keeping-the-evaluation-fair)).

### Retrain or improve the network (Python)

```bash
pip install -r scripts/requirements.txt          # numpy, soundfile, torch, scipy, onnx
python scripts/train_dnn.py --speech speech_clean \
       --noise noise_defence_synthetic noise_general --out my_model --steps 20000
```

The **first folder after `--noise` is the "defence" folder** (70 % of
examples); the others are everyday noise. The script writes
`my_model/defence_gru.mat` (copy it into the repository's `models/` folder
to use it in MATLAB), `defence_gru.onnx` (Jetson / TensorRT) and
`defence_gru.npz`.

### Add your own recordings

The best way to improve the model is **real data**:

* **Real defence noise** (firing ranges, vehicles, rotorcraft, radio hiss):
  only the *first* folder after `--noise` counts as defence noise, so make
  one folder with both, for example `noise_defence_all/` with the synthetic
  clips plus your recordings, and pass it first.
* **More speech**, ideally from the people who will use the system: Indian
  accents, Hindi and other languages, shouted commands, headset or throat
  microphones. `--speech` takes one folder, so add the files to
  `speech_clean/` or to a combined copy of it.

The loaders only accept **16 kHz** files (mono, or stereo, which is averaged
to mono), at least 0.25 s long for speech and 0.5 s for noise. **Other files
are skipped without a warning**, so convert first, for example in MATLAB:

```matlab
[x, fs] = read_audio('recording.mp3');          % from the repository's src/utils
x = resample_k(mean(x, 2), 16000, fs);          % to 16 kHz mono
audiowrite('noise_defence_all/range_day1.wav', 0.99 * x / max(abs(x)), 16000);
```

**More synthetic noise:** use a new starting seed, so the clips do not repeat
existing ones:

```matlab
make_noise_bank('noise_defence_more', 200, 12, 300000);   % 11 types x 200 clips
```

### Get the MS-SNSD files straight from the source

If you only have zip 01 of the package, `scripts/download_mssnsd_subset.py`
downloads the speech and everyday-noise files from Microsoft's GitHub
repository (a fixed version, about 0.9 GB) and checks that every file is
sample-for-sample identical to the training data. Together with
`make_noise_bank` (above), this rebuilds the whole package.

```bash
python scripts/download_mssnsd_subset.py            # add --format wav for WAV files
```

---

## 7. Keeping the evaluation fair

A test only means something if the network has not heard the test material
before. The numbers in the main README come from material **outside** this
training set:

* **Test noise** in `main_defence_anc.m` is generated with seeds below 100.
  Training noise used seeds 101,001 to 111,060, so every test clip is new
  to the network.
* **Test speech** is from CMU ARCTIC (`audio/clean/` in the repository), a
  different corpus with different speakers.
* **The two recordings supplied with the problem statement** were never used
  for training. We only listened to them and measured them to tune the
  noise generator.
* **Real-noise checks** used ESC-50 recordings (helicopter, fireworks,
  sirens, engines). They were never used for training and are not included
  here, because their licence forbids commercial use.

---

## 8. Limitations, honestly

* **Synthetic is not real.** The generator captures the main features of
  each sound, but real weapons, vehicles and rotorcraft vary far more. On
  real helicopter and fireworks recordings the model still improved
  quality (PESQ-WB 1.05 → 1.20 and 1.56 → 1.66) but slightly reduced
  intelligibility (STOI 0.87 → 0.82 and 0.92 → 0.89). Real recordings are
  the most valuable thing to add.
* **The speech is studio speech.** Read English sentences, mostly British
  accents, good microphones. Real users speak other accents and languages,
  shout over noise, and use headsets and radios that colour the sound.
* **It is small.** 3.8 hours of speech from 58 speakers is enough for a
  proof of concept. Production systems train on hundreds of hours.
* **Background voices count as noise.** Because of the babble files, the
  network turns down other people talking. In a headset this is usually what
  you want, but a second nearby speaker will be reduced too.
* **16 kHz only.** Sounds above 8 kHz are not represented.

---

## 9. Licences and credits

Short version (the full text, with citations, is in `LICENSES.md`):

| Folder | Licence | What you must do when you share it |
|---|---|---|
| `speech_clean/` | CC BY 4.0 | Credit the Edinburgh database (citation in `LICENSES.md`) |
| `noise_defence_synthetic/` | Our own work, no third-party audio | Nothing; choose a licence if you publish it |
| `noise_general/` | CC BY-SA 3.0 (applied to the whole folder, to be safe) | Credit DEMAND and MS-SNSD; share changes under the same licence |
| `example_pairs/` | CC BY-SA 3.0 (they contain the above) | Credit as for the speech and the everyday noise |

* MS-SNSD: C. K. A. Reddy et al., "A scalable noisy speech dataset and online
  subjective test framework," *Interspeech*, 2019. Code under the MIT licence.
* Speech: C. Valentini-Botinhao, *Noisy speech database for training speech
  enhancement algorithms and TTS models*, University of Edinburgh, CSTR,
  2017, doi:10.7488/ds/2117.
* DEMAND: J. Thiemann, N. Ito, E. Vincent, "The Diverse Environments
  Multi-channel Acoustic Noise Database (DEMAND)," *Proc. Meetings on
  Acoustics* 19, 2013, doi:10.5281/zenodo.1227121.
* Freesound: <https://freesound.org> (CC0 files only).

---

## 10. Reference: folders and manifest columns

```
defence_anc_training_data/
├── README.md, DATASET_GUIDE.pdf     this guide
├── LICENSES.md                      licences, attribution, citations
├── speech_clean/                    4,639 FLAC files   p234_018.flac ...
├── noise_defence_synthetic/           660 FLAC files   gunshot_001.flac ... battlefield_060.flac
├── noise_general/                     179 FLAC files   noise_train_Babble_3.flac ...
├── example_pairs/
│   ├── clean/, noisy/                 160 + 160 FLAC files
│   └── pairs.csv
├── manifests/
│   ├── speech_clean.csv
│   ├── noise_defence_synthetic.csv
│   ├── noise_general.csv
│   ├── checksums_sha256.txt         SHA-256 of every file in the package
│   └── zip_contents.csv             which zip file contains which file
└── scripts/
    ├── defence_noise.m              the defence-noise generator
    ├── make_noise_bank.m            builds noise_defence_synthetic/
    ├── mix_at_snr.m                 mixes speech and noise at a chosen SNR
    ├── make_training_pairs.py       writes noisy/clean pairs with the training recipe
    ├── train_dnn.py                 trains the network
    ├── download_mssnsd_subset.py    re-downloads the MS-SNSD files and checks them
    ├── verify_dataset.py            checks the unzipped package
    ├── flac_to_wav.m, flac_to_wav.py
    └── requirements.txt
```

In the GitHub repository, `make_training_pairs.py` and `train_dnn.py` are
in `tools/`, and the `.m` files in `src/dataset/`.

**Manifest columns**

| Column | In | Meaning |
|---|---|---|
| `file` | all | file name inside the folder |
| `speaker`, `utterance` | speech | VCTK speaker ID and sentence number |
| `type`, `noise_class`, `clip`, `seed`, `how_made` | defence noise | noise type, impulsive / non-stationary / stationary / mixture, clip number, random seed, and the exact MATLAB command that made the file |
| `category`, `mssnsd_split` | everyday noise | MS-SNSD category and folder (`train` / `test`) |
| `probable_source` | everyday noise | DEMAND (name matches a DEMAND environment) or Freesound |
| `mssnsd_path` | speech, everyday noise | path of the original file in the MS-SNSD repository |
| `duration_s`, `samples` | all | length in seconds and in samples (16 kHz) |
| `rms_dbfs`, `peak_dbfs` | all | average and peak level relative to digital full scale |
| `licence` | all | licence of the file |
| `flac_sha256` | all | checksum of the FLAC file |
| `pcm_sha256` | all | checksum of the audio samples themselves (16-bit little-endian). It stays the same if the file is converted to WAV, which is how `download_mssnsd_subset.py` proves a re-download is identical |
