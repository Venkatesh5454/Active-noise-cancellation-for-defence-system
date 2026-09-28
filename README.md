# Running the noise canceller on an NVIDIA Jetson

A Jetson (Orin Nano / Orin NX / AGX Orin) is a small **Linux computer**: ARM CPU
plus an NVIDIA GPU. It is not a DSP chip, so nothing is flashed to it. You copy a
program onto it and run it, just like on a PC.

MATLAB itself does not run on the Jetson's ARM processor. `anc_jetson.py`
therefore runs **the same algorithm** as the MATLAB code in Python:

* 32 ms sqrt-Hann frames with an 8 ms hop,
* the SPP-MMSE noise tracker,
* the same trained GRU network (`models/defence_gru.onnx` / `.npz`),
* overlap-add synthesis.

Its output was verified to be identical to the MATLAB output (difference
< 3·10⁻⁶ on the sample recordings).

```
USB headset / mic ──► ALSA ──► anc_jetson.py (8 ms blocks) ──► ALSA ──► headphones / radio
                               ├─ STFT + noise tracker   (numpy)
                               └─ GRU mask network       (ONNX Runtime CPU, or GPU/TensorRT)
```

## 1. Prepare the Jetson (once)

1. Install **JetPack** (Ubuntu + CUDA + TensorRT) with NVIDIA SDK Manager, or
   from the SD-card image for the Orin Nano developer kit. Finish the Ubuntu
   first-boot setup and connect the board to the network.
2. Select maximum performance:
   ```bash
   sudo nvpmodel -m 0        # MAXN power mode
   sudo jetson_clocks        # lock clocks at maximum
   ```
3. Install the audio libraries and the Python packages:
   ```bash
   sudo apt update
   sudo apt install -y python3-pip libportaudio2 libsndfile1 ffmpeg git
   pip3 install numpy soundfile sounddevice onnxruntime
   ```
   If `onnxruntime` cannot be installed, the program still runs with its
   built-in NumPy backend (`--backend numpy`).

## 2. Copy the project onto the Jetson

Either clone it on the Jetson:
```bash
git clone -b claude/ai-ml-adaptive-noise-cancellation-4afmlg \
    https://github.com/Venkatesh5454/Active-noise-cancellation-for-defence-system.git anc
```
or copy it from your laptop (replace the IP address):
```bash
scp -r jetson models audio  user@192.168.1.50:~/anc/
```
The program expects this layout: `~/anc/jetson/anc_jetson.py`,
`~/anc/models/defence_gru.onnx` and `~/anc/models/defence_gru.npz`.

## 3. Test with a recording (no microphone needed)

```bash
cd ~/anc/jetson
python3 anc_jetson.py --file ../audio/noisy/defence_sound.mp3 --out cleaned.wav
aplay cleaned.wav
```

## 4. Run live: microphone in, headphones out

1. Plug in a USB headset or a USB sound card with a microphone. This is the
   easiest option; ALSA detects it automatically.
2. Find the device numbers:
   ```bash
   python3 anc_jetson.py --list-devices      # or: arecord -l ; aplay -l
   ```
3. Start the noise canceller, using the device index from the list:
   ```bash
   python3 anc_jetson.py --live --in-dev 11 --out-dev 11 --record demo
   ```
   Speak into the microphone and listen on the headphones. Press **Enter** to
   stop. With `--record demo`, `demo_noisy.wav` and `demo_enhanced.wav` are saved
   for your presentation.

## 5. Check real-time performance

```bash
python3 anc_jetson.py --bench          # compute time per 8 ms block
sudo tegrastats                        # CPU / GPU load, power, temperature
```
Each 8 ms block must be processed in less than 8 ms. On a desktop x86 CPU,
one core needs about 0.3 ms per block. The Jetson's ARM cores are slower,
which still leaves a large margin; measure it on your board with `--bench`.

## 6. Optional: GPU / TensorRT

The problem statement asks for ONNX / TensorRT conversion.

* **Convert the network to a TensorRT engine.** `trtexec` ships with JetPack
  and also reports the inference latency:
  ```bash
  /usr/src/tensorrt/bin/trtexec --onnx=../models/defence_gru.onnx \
      --saveEngine=defence_gru_fp16.plan --fp16
  ```
* **Run the program on the GPU** by installing the Jetson build of
  `onnxruntime-gpu` that matches your JetPack version (listed on NVIDIA's
  Jetson Zoo / Jetson AI Lab pages), then:
  ```bash
  python3 anc_jetson.py --live --backend trt --in-dev 11 --out-dev 11
  ```
  The program uses the TensorRT or CUDA execution provider when available, and
  the CPU otherwise.

This network is small (0.5 M parameters, one frame at a time), so the CPU
usually gives the *lowest latency*: sending such small jobs to the GPU costs
more time than the computation itself. The GPU / TensorRT path pays off for
larger networks or many channels at once. This path has not been tested on a
real Jetson; the CPU path is the one verified against MATLAB.

## 7. Optional: start automatically at boot

Create `/etc/systemd/system/anc.service`:
```ini
[Unit]
Description=Defence ANC
After=sound.target

[Service]
User=user
WorkingDirectory=/home/user/anc/jetson
ExecStart=/usr/bin/python3 anc_jetson.py --live --in-dev 11 --out-dev 11
Restart=always

[Install]
WantedBy=multi-user.target
```
Adjust `User` and the paths, then enable it with `sudo systemctl enable --now anc`.
Without a keyboard attached, as under systemd, live mode runs until the
service is stopped.

## Other routes

* **MATLAB → C/C++ on the Jetson.** Use MATLAB Coder with the *MATLAB Coder
  Support Package for NVIDIA Jetson and NVIDIA DRIVE Platforms*, which needs
  those licences. `anc_process_frame.m` is already a fixed-size, frame-by-frame
  function. For code generation, the model weights must be loaded with
  `coder.load`, and the state struct must have a fixed set of fields.
* **A real DSP board (TI / Analog Devices).** Port `anc_process_frame` and
  `dnn_mask_step` to C. It uses about 0.5 M multiply-accumulates per 8 ms
  frame, plus one 512-point FFT and one inverse FFT.
