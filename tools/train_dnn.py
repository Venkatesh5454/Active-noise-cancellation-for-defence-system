#!/usr/bin/env python3
"""
Train the causal GRU mask estimator used by the MATLAB noise canceller
(AI/ML stage of src/anc/defence_anc.m).

    python tools/train_dnn.py --speech DIR --noise DIR [DIR ...] --out models/

Model (per 8 ms frame, 32 ms window, 16 kHz, no look-ahead):
    input  : [log|Y|^2 , log lambda_s]   (257 + 257, globally normalised)
             lambda_s = SPP-MMSE stationary-noise PSD (same tracker as MATLAB)
    dense  : 514 -> H1, tanh
    GRU    : H1 -> H
    dense  : H -> 257, sigmoid              => spectral gain (mask)
Loss: power-law compressed magnitude + complex MSE (Braun & Tashev, 2020).

Training data are mixed on the fly: clean speech (e.g. MS-SNSD / VCTK /
LibriSpeech) + defence noise bank (generated with src/dataset/defence_noise.m)
+ any other noise recordings, SNR -10 ... +20 dB, random level / EQ /
reverberation.  The trained weights are exported to <out>/defence_gru.mat
(loaded by MATLAB) and <out>/defence_gru.onnx (Jetson / TensorRT).
"""
import argparse, glob, math, os, random, time
import numpy as np
import soundfile as sf
import torch
import torch.nn as nn

FS, N, R = 16000, 512, 128
K = N // 2 + 1


# --------------------------------------------------------------------------
#  front end identical to anc_init.m / anc_process_frame.m
# --------------------------------------------------------------------------
WIN = torch.sqrt(0.5 - 0.5 * torch.cos(2 * math.pi * torch.arange(N) / N)).double()


def stft(x):
    """x: (B, T) float64 -> complex (B, F, K); streaming alignment."""
    x = torch.nn.functional.pad(x, (N - R, 0))
    fr = x.unfold(-1, N, R) * WIN
    return torch.fft.rfft(fr, dim=-1)


def istft(Y, length):
    fr = torch.fft.irfft(Y, n=N, dim=-1) * WIN            # (B, F, N)
    B, F, _ = fr.shape
    out = torch.zeros(B, (F - 1) * R + N, dtype=fr.dtype)
    for i in range(N // R):                                # overlap-add
        seg = fr[:, :, i * R:(i + 1) * R].reshape(B, -1)
        out[:, i * R:i * R + F * R] += seg
    out = out * 0.5                                        # R / sum(win^2)
    return out[:, N - R:N - R + length]


def spp_tracker(P, hop_ms=8.0):
    """SPP-MMSE noise PSD (Gerkmann & Hendriks 2012), P: (B, F, K) float64."""
    hr = hop_ms / 16.0
    a_psd, a_p = 0.8 ** hr, 0.9 ** hr
    xi = 10 ** 1.5
    lg, ge = math.log(1 / (1 + xi)), xi / (1 + xi)
    n_init = 8
    B, F, _ = P.shape
    lam = torch.zeros(B, K, dtype=P.dtype)
    pm = torch.full((B, K), 0.5, dtype=P.dtype)
    out = torch.empty_like(P)
    for t in range(F):
        p = P[:, t]
        if t < n_init:
            lam = lam + (p - lam) / (t + 1)
        else:
            glr = torch.exp(torch.clamp(lg + ge * p / lam, max=200.0))
            ph1 = glr / (1 + glr)
            pm = a_p * pm + (1 - a_p) * ph1
            ph1 = torch.where(pm > 0.99, torch.clamp(ph1, max=0.99), ph1)
            lam = a_psd * lam + (1 - a_psd) * (ph1 * lam + (1 - ph1) * p)
        out[:, t] = torch.clamp(lam, min=1e-12)
    return out


def features(P, lam):
    return torch.cat([torch.log(P + 1e-10), torch.log(lam + 1e-10)], dim=-1)


# --------------------------------------------------------------------------
class MaskGRU(nn.Module):
    def __init__(self, h1=192, h=256):
        super().__init__()
        self.register_buffer("mu", torch.zeros(2 * K))
        self.register_buffer("sd", torch.ones(2 * K))
        self.fc1 = nn.Linear(2 * K, h1)
        self.gru = nn.GRU(h1, h, batch_first=True)
        self.fc2 = nn.Linear(h, K)

    def forward(self, f, h0=None):
        z = torch.tanh(self.fc1((f - self.mu) / self.sd))
        y, hn = self.gru(z, h0)
        return torch.sigmoid(self.fc2(y)), hn


# --------------------------------------------------------------------------
#  on-the-fly data generation
# --------------------------------------------------------------------------
def load_dir(d, min_len):
    out = []
    for f in sorted(glob.glob(os.path.join(d, "*.wav"))):
        x, fs = sf.read(f, dtype="float32", always_2d=True)
        x = x.mean(1)
        if fs != FS or len(x) < min_len:
            continue
        x = x - x.mean()
        r = np.sqrt(np.mean(x ** 2))
        if r > 1e-6:
            out.append((x / r).astype(np.float32))
    return out


def rand_seg(pool, L, allow_pause=False):
    x = pool[random.randrange(len(pool))]
    if len(x) >= L:
        s = random.randrange(len(x) - L + 1)
        return x[s:s + L].copy()
    out = np.zeros(L, np.float32)
    pos = 0 if not allow_pause else random.randrange(0, max(1, L - len(x)))
    while pos < L:
        n = min(len(x), L - pos)
        out[pos:pos + n] = x[:n]
        pos += n + (random.randrange(1600, 8000) if allow_pause else 0)
        x = pool[random.randrange(len(pool))]
    return out


def rand_rir():
    t60 = random.uniform(0.15, 0.6)
    L = int(t60 * FS)
    t = np.arange(L) / FS
    h = np.random.randn(L) * np.exp(-6.9 * t / t60)
    h[0] = 1.0 / random.uniform(0.3, 1.0)       # direct path
    return (h / np.sqrt(np.sum(h ** 2))).astype(np.float32)


def tilt(x):
    """random spectral tilt (+-4 dB/oct) with a 1st-order shelf"""
    a = random.uniform(-0.5, 0.5)
    return (x - a * np.concatenate([[0.0], x[:-1]])).astype(np.float32)


def make_batch(speech, noise_def, noise_gen, B, L):
    S, X = np.zeros((B, L), np.float32), np.zeros((B, L), np.float32)
    for b in range(B):
        s = rand_seg(speech, L, allow_pause=True)
        if random.random() < 0.15:
            s = np.convolve(s, rand_rir())[:L]
        if random.random() < 0.3:
            s = tilt(s)
        if random.random() < 0.05:
            s[:] = 0.0                                   # noise only
        pool = noise_def if (random.random() < 0.7 or not noise_gen) else noise_gen
        n = rand_seg(pool, L)
        if random.random() < 0.35:
            pool2 = noise_gen if (noise_gen and random.random() < 0.6) else noise_def
            n = n + random.uniform(0.2, 1.0) * rand_seg(pool2, L)
        if random.random() < 0.3:
            n = tilt(n)
        snr = random.uniform(-10, 20) if random.random() < 0.95 else 40.0
        ps = np.mean(s ** 2)
        pn = np.mean(n ** 2) + 1e-12
        if ps > 0:
            n = n * np.sqrt(ps / (pn * 10 ** (snr / 10)))
        else:
            n = n * np.sqrt(0.0025 / pn)
        x = s + n
        g = 0.05 * 10 ** (random.uniform(-20, 12) / 20)   # random input level
        k = g / (np.sqrt(np.mean(x ** 2)) + 1e-9)
        S[b], X[b] = s * k, x * k
    return torch.from_numpy(S).double(), torch.from_numpy(X).double()


def loss_fn(M, Y, Sref, c=0.3):
    Yh = M * Y
    Am, As = torch.abs(Yh) + 1e-8, torch.abs(Sref) + 1e-8
    Ym, Sm = Am ** c, As ** c
    lmag = torch.mean((Ym - Sm) ** 2)
    lcpx = torch.mean(torch.abs(Ym * Yh / Am - Sm * Sref / As) ** 2)
    return 0.7 * lmag + 0.3 * lcpx


# --------------------------------------------------------------------------
def export(model, out_dir):
    import scipy.io
    os.makedirs(out_dir, exist_ok=True)
    sd = {k: v.detach().cpu().double().numpy() for k, v in model.state_dict().items()}
    mat = {
        "mu": sd["mu"], "sd": sd["sd"],
        "W1": sd["fc1.weight"], "b1": sd["fc1.bias"],
        "Wih": sd["gru.weight_ih_l0"], "Whh": sd["gru.weight_hh_l0"],
        "bih": sd["gru.bias_ih_l0"], "bhh": sd["gru.bias_hh_l0"],
        "W2": sd["fc2.weight"], "b2": sd["fc2.bias"],
        "frame": N, "hop": R, "fs": FS,
    }
    scipy.io.savemat(os.path.join(out_dir, "defence_gru.mat"), mat, do_compression=True)
    np.savez_compressed(os.path.join(out_dir, "defence_gru.npz"),        # jetson numpy backend
                        **{k: np.asarray(v, np.float64) for k, v in mat.items()
                           if k not in ("frame", "hop", "fs")})

    class Step(nn.Module):                      # one frame, explicit state
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, f, h):
            z = torch.tanh(self.m.fc1((f - self.m.mu) / self.m.sd))
            y, hn = self.m.gru(z.unsqueeze(1), h)
            return torch.sigmoid(self.m.fc2(y[:, 0])), hn

    step = Step(model).float().eval()
    f = torch.zeros(1, 2 * K)
    h = torch.zeros(1, 1, model.gru.hidden_size)
    torch.onnx.export(step, (f, h), os.path.join(out_dir, "defence_gru.onnx"),
                      input_names=["features", "h_in"], output_names=["mask", "h_out"],
                      opset_version=14, dynamo=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speech", required=True)
    ap.add_argument("--noise", nargs="+", required=True,
                    help="first dir = defence noise bank, others = general noise")
    ap.add_argument("--out", default="models")
    ap.add_argument("--steps", type=int, default=6000)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--h1", type=int, default=192)
    ap.add_argument("--h", type=int, default=256)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed)

    L = int(a.seconds * FS)
    speech = load_dir(a.speech, 4000)
    noise_def = load_dir(a.noise[0], 8000)
    noise_gen = [x for d in a.noise[1:] for x in load_dir(d, 8000)]
    print(f"speech files {len(speech)}, defence noise {len(noise_def)}, other noise {len(noise_gen)}")

    model = MaskGRU(a.h1, a.h)
    # feature normalisation statistics
    with torch.no_grad():
        acc = []
        for _ in range(8):
            _, X = make_batch(speech, noise_def, noise_gen, a.batch, L)
            Y = stft(X); P = Y.real ** 2 + Y.imag ** 2
            acc.append(features(P, spp_tracker(P)).reshape(-1, 2 * K))
        acc = torch.cat(acc)
        model.mu.copy_(acc.mean(0).float()); model.sd.copy_(acc.std(0).float() + 1e-3)
    print("params:", sum(p.numel() for p in model.parameters()))

    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.steps, pct_start=0.05)
    t0, run = time.time(), 0.0
    for step in range(1, a.steps + 1):
        Sc, X = make_batch(speech, noise_def, noise_gen, a.batch, L)
        with torch.no_grad():
            Y = stft(X); Sref = stft(Sc)
            P = Y.real ** 2 + Y.imag ** 2
            F = features(P, spp_tracker(P)).float()
        M, _ = model(F)
        loss = loss_fn(M.double(), Y, Sref)
        opt.zero_grad(); loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step(); sched.step()
        run = 0.98 * run + 0.02 * loss.item() if step > 1 else loss.item()
        if step % 100 == 0:
            print(f"step {step:5d}  loss {run:.5f}  lr {sched.get_last_lr()[0]:.2e}  "
                  f"{(time.time() - t0) / step:.2f} s/step", flush=True)
        if step % 1000 == 0 or step == a.steps:
            torch.save(model.state_dict(), os.path.join(a.out, "defence_gru.pt"))
    export(model, a.out)
    print("exported to", a.out)


if __name__ == "__main__":
    main()
