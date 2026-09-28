#!/usr/bin/env python3
"""
Real-time defence noise canceller for NVIDIA Jetson (runs on any Linux / Windows PC too).

Same algorithm and same numbers as the MATLAB code
(src/anc/defence_anc.m -> anc_process_frame.m -> dnn_mask_step.m):
    16 kHz, 32 ms sqrt-Hann frames, 8 ms hop, SPP-MMSE noise tracker,
    GRU mask network (models/defence_gru.onnx or .npz), -30 dB mask floor,
    70 Hz spectral low-cut, overlap-add.  Causal, 32 ms algorithmic latency.

Usage
    python3 anc_jetson.py --file noisy.mp3 --out enhanced.wav     # offline file
    python3 anc_jetson.py --list-devices                          # sound cards
    python3 anc_jetson.py --live --in-dev 11 --out-dev 11         # mic -> headphones
    python3 anc_jetson.py --live --record demo                    # + save noisy/clean wav
    python3 anc_jetson.py --bench                                 # time per 8 ms block
Backends:  --backend onnx (ONNX Runtime CPU, default) | trt (GPU: TensorRT/CUDA
           execution provider) | numpy (no extra packages)
"""
import argparse, math, os, sys, time
import numpy as np

FS, N, R = 16000, 512, 128          # sample rate, frame (32 ms), hop (8 ms)
K = N // 2 + 1                      # 257 frequency bins
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "..", "models")


# ---------------------------------------------------------------------------
#  GRU mask network backends
# ---------------------------------------------------------------------------
class NumpyGRU:
    """dense(tanh) -> GRU -> dense(sigmoid); identical to dnn_mask_step.m"""
    def __init__(self, path):
        w = np.load(path)
        self.mu, self.sd = w["mu"], w["sd"]
        self.W1, self.b1 = w["W1"], w["b1"]
        self.Wih, self.Whh, self.bih, self.bhh = w["Wih"], w["Whh"], w["bih"], w["bhh"]
        self.W2, self.b2 = w["W2"], w["b2"]
        self.H = self.Whh.shape[1]
        self.reset()

    def reset(self):
        self.h = np.zeros(self.H)

    def __call__(self, f):
        H = self.H
        z = np.tanh(self.W1 @ ((f - self.mu) / self.sd) + self.b1)
        gi = self.Wih @ z + self.bih
        gh = self.Whh @ self.h + self.bhh
        r = 1.0 / (1.0 + np.exp(-(gi[:H] + gh[:H])))
        u = 1.0 / (1.0 + np.exp(-(gi[H:2 * H] + gh[H:2 * H])))
        n = np.tanh(gi[2 * H:] + r * gh[2 * H:])
        self.h = (1.0 - u) * n + u * self.h
        return 1.0 / (1.0 + np.exp(-(self.W2 @ self.h + self.b2)))


class OnnxGRU:
    """Same network through ONNX Runtime (CPU, CUDA or TensorRT)."""
    def __init__(self, path, gpu=False):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1           # tiny model: 1 thread = lowest jitter
        so.inter_op_num_threads = 1
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        prov = ["CPUExecutionProvider"]
        if gpu:
            avail = ort.get_available_providers()
            prov = []
            if "TensorrtExecutionProvider" in avail:
                os.makedirs(os.path.join(HERE, "trt_cache"), exist_ok=True)
                prov.append(("TensorrtExecutionProvider",
                             {"trt_fp16_enable": True, "trt_engine_cache_enable": True,
                              "trt_engine_cache_path": os.path.join(HERE, "trt_cache")}))
            if "CUDAExecutionProvider" in avail:
                prov.append("CUDAExecutionProvider")
            prov.append("CPUExecutionProvider")
        self.sess = ort.InferenceSession(path, so, providers=prov)
        self.H = self.sess.get_inputs()[1].shape[2]
        print("ONNX Runtime providers:", self.sess.get_providers())
        self.reset()

    def reset(self):
        self.h = np.zeros((1, 1, self.H), np.float32)

    def __call__(self, f):
        m, self.h = self.sess.run(None, {"features": f.astype(np.float32)[None],
                                         "h_in": self.h})
        return m[0].astype(np.float64)


# ---------------------------------------------------------------------------
#  streaming noise canceller (one 8 ms block in -> one 8 ms block out)
# ---------------------------------------------------------------------------
class ANCEngine:
    def __init__(self, net, mask_floor_db=-30.0, lowcut_hz=70.0, lowcut_db=-30.0):
        self.net = net
        n = np.arange(N)
        self.win = np.sqrt(0.5 - 0.5 * np.cos(2 * np.pi * n / N))   # sqrt-Hann
        self.ola = R / np.sum(self.win ** 2)                          # 0.5
        f = np.arange(K) * FS / N
        self.lowcut = np.where(f < lowcut_hz, 10 ** (lowcut_db / 20), 1.0)
        self.floor = 10 ** (mask_floor_db / 20)
        # SPP-MMSE noise tracker constants (Gerkmann & Hendriks 2012), 8 ms hop
        xi = 10 ** 1.5
        self.lg, self.ge = math.log(1 / (1 + xi)), xi / (1 + xi)
        self.a_psd, self.a_p = 0.8 ** 0.5, 0.9 ** 0.5
        self.n_init = 8
        self.dc_a = math.exp(-2 * math.pi * 10 / FS)                  # DC blocker
        self.reset()

    def reset(self):
        self.inbuf = np.zeros(N)
        self.outbuf = np.zeros(N)
        self.lam = np.zeros(K)
        self.pm = np.full(K, 0.5)
        self.frame = 0
        self.dc_x = 0.0
        self.dc_y = 0.0
        self.net.reset()

    def dc_block(self, x):
        y = np.empty_like(x)
        px, py, a = self.dc_x, self.dc_y, self.dc_a
        for i, v in enumerate(x):              # y[n] = x[n] - x[n-1] + a*y[n-1]
            py = v - px + a * py
            px = v
            y[i] = py
        self.dc_x, self.dc_y = px, py
        return y

    def process_block(self, x):
        """x: 128 new samples (float, 16 kHz) -> 128 enhanced samples"""
        x = self.dc_block(np.asarray(x, np.float64))
        self.frame += 1
        self.inbuf[:-R] = self.inbuf[R:]
        self.inbuf[-R:] = x
        Y = np.fft.rfft(self.inbuf * self.win)
        P = np.maximum(Y.real ** 2 + Y.imag ** 2, 1e-12)
        # stationary noise tracker
        if self.frame <= self.n_init:
            self.lam += (P - self.lam) / self.frame
        else:
            glr = np.exp(np.minimum(self.lg + self.ge * P / self.lam, 200.0))
            ph1 = glr / (1 + glr)
            self.pm = self.a_p * self.pm + (1 - self.a_p) * ph1
            ph1 = np.where(self.pm > 0.99, np.minimum(ph1, 0.99), ph1)
            self.lam = self.a_psd * self.lam + (1 - self.a_psd) * (ph1 * self.lam + (1 - ph1) * P)
        lam = np.maximum(self.lam, 1e-12)
        # AI/ML mask
        feat = np.concatenate([np.log(P + 1e-10), np.log(lam + 1e-10)])
        G = np.maximum(self.net(feat), self.floor) * self.lowcut
        # synthesis (overlap-add)
        self.outbuf += np.fft.irfft(G * Y, n=N) * self.win
        out = self.outbuf[:R] * self.ola
        self.outbuf[:-R] = self.outbuf[R:]
        self.outbuf[-R:] = 0.0
        return out


# ---------------------------------------------------------------------------
#  helpers
# ---------------------------------------------------------------------------
def resample_k(x, fs_out, fs_in):
    """Kaiser-windowed sinc polyphase resampler (same as src/utils/resample_k.m)."""
    if fs_out == fs_in:
        return np.asarray(x, np.float64).copy()
    g = math.gcd(int(fs_out), int(fs_in))
    p, q = int(fs_out) // g, int(fs_in) // g
    rej, fc = 60.0, 1.0 / (2 * max(p, q))
    L = math.ceil((rej - 8) / (28.714 * fc / 10))
    t = np.arange(-L, L + 1)
    beta, M = 0.1102 * (rej - 8.7), 2 * L + 1
    kais = np.i0(beta * np.sqrt(1 - (2 * np.arange(M) / (M - 1) - 1) ** 2)) / np.i0(beta)
    h = kais * 2 * p * fc * np.sinc(2 * fc * t)
    h = p * h / h.sum()
    x = np.asarray(x, np.float64)
    Lx, Lh = len(x), len(h)
    Ly = math.ceil(Lx * p / q)
    tt = np.arange(Ly) * q + L
    nmax = tt // p
    r = tt - nmax * p
    J = math.ceil(Lh / p)
    hp = np.zeros(p * J)
    hp[:Lh] = h
    H = hp.reshape(J, p).T                       # H[r, j] = h[r + j*p]
    y = np.zeros(Ly)
    for j in range(J):
        idx = nmax - j
        ok = (idx >= 0) & (idx < Lx)
        y[ok] += H[r[ok], j] * x[idx[ok]]
    return y


def load_net(backend):
    onnx_path = os.path.join(MODEL_DIR, "defence_gru.onnx")
    npz_path = os.path.join(MODEL_DIR, "defence_gru.npz")
    if backend in ("onnx", "trt"):
        try:
            return OnnxGRU(onnx_path, gpu=(backend == "trt"))
        except ImportError:
            print("onnxruntime not installed -> using the numpy backend")
    return NumpyGRU(npz_path)


def read_audio(path):
    """soundfile first; ffmpeg as fall-back for formats libsndfile cannot decode"""
    import soundfile as sf
    try:
        return sf.read(path, dtype="float64", always_2d=True)
    except Exception:
        import json, subprocess
        info = json.loads(subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "a:0", "-show_entries",
             "stream=sample_rate,channels", "-of", "json", path],
            capture_output=True, check=True).stdout)["streams"][0]
        fs, ch = int(info["sample_rate"]), int(info["channels"])
        raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-f", "f32le", "-"],
                             capture_output=True, check=True).stdout
        return np.frombuffer(raw, np.float32).astype(np.float64).reshape(-1, ch), fs


def enhance_file(eng, path, out_path):
    import soundfile as sf
    x, fs = read_audio(path)
    if x.shape[1] >= 2 and np.allclose(x[:, 0], x[:, 1], atol=1e-4):
        x = x.mean(axis=1)                       # dual-mono file
    else:
        if x.shape[1] >= 2:
            print("2 different channels: using channel 1 (dual-mic NLMS mode is in the MATLAB code)")
        x = x[:, 0]
    x = resample_k(x, FS, fs)
    Lx, delay = len(x), N - R
    nhop = math.ceil((Lx + delay) / R)
    xin = np.concatenate([x, np.zeros(nhop * R - Lx)])
    y = np.zeros(nhop * R)
    eng.reset()
    t0 = time.perf_counter()
    for b in range(nhop):
        y[b * R:(b + 1) * R] = eng.process_block(xin[b * R:(b + 1) * R])
    dt = time.perf_counter() - t0
    y = y[delay:delay + Lx]
    pk = np.max(np.abs(y))
    if pk > 0.95:
        y *= 0.95 / pk
    sf.write(out_path, y, FS, subtype="PCM_16")
    print(f"{path}: {Lx / FS:.1f} s processed in {dt:.2f} s (real-time factor {dt / (Lx / FS):.3f})"
          f" -> {out_path}")


def live(eng, in_dev, out_dev, record):
    import sounddevice as sd
    rec_in, rec_out, stats = [], [], {"n": 0, "t": 0.0, "max": 0.0, "xruns": 0}

    def cb(indata, outdata, frames, tinfo, status):
        if status:
            stats["xruns"] += 1
        t0 = time.perf_counter()
        y = eng.process_block(indata[:, 0])
        dt = time.perf_counter() - t0
        outdata[:, 0] = np.clip(y, -1, 1)
        stats["n"] += 1; stats["t"] += dt; stats["max"] = max(stats["max"], dt)
        if record:
            rec_in.append(indata[:, 0].copy()); rec_out.append(y.astype(np.float32))

    eng.reset()
    with sd.Stream(device=(in_dev, out_dev), samplerate=FS, blocksize=R, channels=1,
                   dtype="float32", latency="low", callback=cb) as s:
        print(f"LIVE noise cancellation running (block 8 ms, algorithmic latency 32 ms,"
              f" device latency in/out {1000 * s.latency[0]:.0f}/{1000 * s.latency[1]:.0f} ms)."
              f"  Press Enter to stop.")
        if sys.stdin is not None and sys.stdin.isatty():
            input()
        else:                                    # started as a service: run until stopped
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
    n = max(stats["n"], 1)
    print(f"blocks {stats['n']}, compute mean {1000 * stats['t'] / n:.2f} ms,"
          f" max {1000 * stats['max']:.2f} ms (budget 8 ms), over/underruns {stats['xruns']}")
    if record:
        import soundfile as sf
        sf.write(record + "_noisy.wav", np.concatenate(rec_in), FS)
        sf.write(record + "_enhanced.wav", np.concatenate(rec_out), FS)
        print(f"saved {record}_noisy.wav and {record}_enhanced.wav")


def bench(eng, blocks=3000):
    rng = np.random.default_rng(0)
    x = 0.05 * rng.standard_normal(blocks * R)
    eng.reset()
    ts = np.empty(blocks)
    for b in range(blocks):
        t0 = time.perf_counter()
        eng.process_block(x[b * R:(b + 1) * R])
        ts[b] = time.perf_counter() - t0
    ts *= 1000
    print(f"per 8 ms block: mean {ts.mean():.3f} ms, 99th pct {np.percentile(ts, 99):.3f} ms,"
          f" max {ts.max():.3f} ms  ->  CPU load {100 * ts.mean() / 8:.1f} % of real time")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file"); ap.add_argument("--out")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--in-dev", type=int); ap.add_argument("--out-dev", type=int)
    ap.add_argument("--record", help="file prefix to save noisy/enhanced audio in live mode")
    ap.add_argument("--list-devices", action="store_true")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--backend", choices=["onnx", "trt", "numpy"], default="onnx")
    ap.add_argument("--floor-db", type=float, default=-30.0, help="lowest mask gain (dB)")
    a = ap.parse_args()

    if a.list_devices:
        import sounddevice as sd
        print(sd.query_devices())
        return
    eng = ANCEngine(load_net(a.backend), mask_floor_db=a.floor_db)
    if a.bench:
        bench(eng)
    if a.file:
        out = a.out or os.path.splitext(os.path.basename(a.file))[0] + "_enhanced.wav"
        enhance_file(eng, a.file, out)
    if a.live:
        live(eng, a.in_dev, a.out_dev, a.record)
    if not (a.bench or a.file or a.live):
        ap.print_help()


if __name__ == "__main__":
    main()
