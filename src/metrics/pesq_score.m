function [mos, raw] = pesq_score(ref, deg, fs, mode)
%PESQ_SCORE  Perceptual Evaluation of Speech Quality (ITU-T P.862 family).
%
%   mos = pesq_score(ref, deg, fs)         wide-band  PESQ, P.862.2 MOS-LQO
%   mos = pesq_score(ref, deg, fs, 'nb')   narrow-band PESQ, P.862 + P.862.1 MOS-LQO
%   [mos, raw] = pesq_score(...)           raw = P.862 raw score (NB only)
%
%   ref : clean reference speech          deg : degraded / enhanced speech
%   fs  : sampling rate.  'wb' works at 16 kHz, 'nb' at 8 or 16 kHz; any
%         other rate is resampled automatically (16 kHz for wb, 8 kHz nb).
%
%   Score range: about -0.5 ... 4.5 (MOS-LQO about 1.0 ... 4.64); higher is
%   better.  The signals do not need to be time aligned - PESQ performs its
%   own level alignment, IRS / wide-band input filtering, utterance-based
%   time-delay estimation and bad-interval re-alignment.
%
%   Independent, toolbox-free MATLAB re-implementation of the algorithm
%   specified in ITU-T Recommendations P.862 (2001), P.862.1 (2003) and
%   P.862.2 (2005).  Its output was cross-checked against the ITU-T ANSI-C
%   reference implementation (see tools/validate_metrics.py).
%
%   IPR NOTICE: the PESQ algorithm is the intellectual property of OPTICOM
%   GmbH and Psytechnics Ltd.  This implementation is provided for research
%   and educational use only (e.g. evaluating an enhancement algorithm in
%   an academic project).  Any commercial use requires a PESQ licence from
%   the IPR owners; for certified results use the ITU-T reference software.

    if nargin < 4 || isempty(mode), mode = 'wb'; end
    mode = lower(mode);
    ref = double(ref(:));
    deg = double(deg(:));
    if strcmp(mode, 'wb')
        if fs ~= 16000
            ref = resample_k(ref, 16000, fs);
            deg = resample_k(deg, 16000, fs);
            fs  = 16000;
        end
    elseif strcmp(mode, 'nb')
        if fs ~= 8000 && fs ~= 16000
            ref = resample_k(ref, 8000, fs);
            deg = resample_k(deg, 8000, fs);
            fs  = 8000;
        end
    else
        error('pesq_score: mode must be ''wb'' or ''nb''.');
    end

    % Scale as the reference wrapper does (the score is level independent).
    mx = max(max(abs(ref)), max(abs(deg)));
    if mx > 0
        ref = ref / mx;
        deg = deg / mx;
    end

    C = pesq_constants(fs, mode);
    raw = NaN;
    mos = NaN;

    r = load_src(ref, C);
    d = load_src(deg, C);
    if (r.Nsamples - 2*C.SB*C.DS < fs/4) || (d.Nsamples - 2*C.SB*C.DS < fs/4)
        warning('pesq_score:short', 'Signals shorter than 0.25 s - PESQ not computed.');
        return;
    end

    maxN = max(r.Nsamples, d.Nsamples);
    r = fix_power_level(r, maxN, C);
    d = fix_power_level(d, maxN, C);

    if strcmp(mode, 'nb')
        r.data = apply_filter(r.data, r.Nsamples, C.IRS_dB, C);
        d.data = apply_filter(d.data, d.Nsamples, C.IRS_dB, C);
    else
        r.data = wb_input_filter(r.data, r.Nsamples, C);
        d.data = wb_input_filter(d.data, d.Nsamples, C);
    end

    model_ref = r.data;
    model_deg = d.data;

    % ---- time alignment (operates on a band-filtered copy) --------------
    r.data = dc_block(r.data, r.Nsamples, C);
    d.data = dc_block(d.data, d.Nsamples, C);
    r.data = sos_filter(C.InIIR, r.data);
    d.data = sos_filter(C.InIIR, d.data);
    [r.VAD, r.logVAD] = apply_vad(r.data, r.Nsamples, C);
    [d.VAD, d.logVAD] = apply_vad(d.data, d.Nsamples, C);

    E = struct();
    E.Crude_DelayEst = crude_delay(r.logVAD, fix(r.Nsamples/C.DS), 0, ...
                                   d.logVAD, fix(d.Nsamples/C.DS), 0) * C.DS;
    E = utterance_locate(r, d, E, C);
    if E.Nutterances < 1
        warning('pesq_score:noutt', 'No utterances detected - PESQ not computed.');
        return;
    end

    % ---- perceptual model on the level-aligned, input-filtered signals ---
    r.data = model_ref;
    d.data = model_deg;
    if numel(r.data) < numel(d.data)
        r.data(end+1:numel(d.data)) = 0;
    elseif numel(d.data) < numel(r.data)
        d.data(end+1:numel(r.data)) = 0;
    end
    raw = psychoacoustic_model(r, d, E, C);

    if strcmp(mode, 'nb')
        mos = 0.999 + 4 / (1 + exp(-1.4945 * raw + 4.6607));
    else
        mos = 0.999 + 4 / (1 + exp(-1.3669 * raw + 3.8224));
        raw = NaN;
    end
end

% =========================================================================
%  Constants and tables (ITU-T P.862 / P.862.2)
% =========================================================================
function C = pesq_constants(fs, mode)
    C.Fs   = fs;
    C.mode = mode;
    C.SB   = 75;                    % SEARCHBUFFER (VAD windows)
    C.DPAD = 320 * (fs / 1000);     % DATAPADDING_MSECS
    C.MINSPEECHLGTH  = 4;
    C.JOINSPEECHLGTH = 50;
    C.MINUTTLENGTH   = 50;
    C.MAXNUTT        = 50;

    C.align_dB = [0 -500; 50 -500; 100 -500; 125 -500; 160 -500; 200 -500; ...
        250 -500; 300 -500; 350 0; 400 0; 500 0; 600 0; 630 0; 800 0; 1000 0; ...
        1250 0; 1600 0; 2000 0; 2500 0; 3000 0; 3250 0; 3500 -500; 4000 -500; ...
        5000 -500; 6300 -500; 8000 -500];
    C.IRS_dB = [0 -200; 50 -40; 100 -20; 125 -12; 160 -6; 200 0; 250 4; ...
        300 6; 350 8; 400 10; 500 11; 600 12; 700 12; 800 12; 1000 12; ...
        1300 12; 1600 12; 2000 12; 2500 12; 3000 12; 3250 12; 3500 4; ...
        4000 -200; 5000 -200; 6300 -200; 8000 -200];

    abs_thresh = [51286152.000000 2454709.500000 70794.593750 4897.788574 ...
        1174.897705 389.045166 104.712860 45.708820 17.782795 9.772372 ...
        4.897789 3.090296 1.905461 1.258925 0.977237 0.724436 0.562341 ...
        0.457088 0.389045 0.331131 0.295121 0.269153 0.257040 0.251189 ...
        0.251189 0.251189 0.251189 0.263027 0.288403 0.309030 0.338844 ...
        0.371535 0.398107 0.436516 0.467735 0.489779 0.501187 0.501187 ...
        0.512861 0.524807 0.524807 0.524807 0.512861 0.478630 0.426580 ...
        0.371535 0.363078 0.416869 0.537032];
    bark_centre = [0.078672 0.316341 0.636559 0.961246 1.290450 1.624217 ...
        1.962597 2.305636 2.653383 3.005889 3.363201 3.725371 4.092449 ...
        4.464486 4.841533 5.223642 5.610866 6.003256 6.400869 6.803755 ...
        7.211971 7.625571 8.044611 8.469146 8.899232 9.334927 9.776288 ...
        10.223374 10.676242 11.134952 11.599563 12.070135 12.546731 ...
        13.029408 13.518232 14.013264 14.514566 15.022202 15.536238 ...
        16.056736 16.583761 17.117382 17.657663 18.204674 18.758478 ...
        19.319147 19.886751 20.461355 21.043034];
    bark_width = [0.157344 0.317994 0.322441 0.326934 0.331474 0.336061 ...
        0.340697 0.345381 0.350114 0.354897 0.359729 0.364611 0.369544 ...
        0.374529 0.379565 0.384653 0.389794 0.394989 0.400236 0.405538 ...
        0.410894 0.416306 0.421773 0.427297 0.432877 0.438514 0.444209 ...
        0.449962 0.455774 0.461645 0.467577 0.473569 0.479621 0.485736 ...
        0.491912 0.498151 0.504454 0.510819 0.517250 0.523745 0.530308 ...
        0.536934 0.543629 0.550390 0.557220 0.564119 0.571085 0.578125 ...
        0.585232];
    pdc = [100.000000 99.999992 100.000000 100.000008 100.000008 100.000015 ...
        99.999992 99.999969 50.000027 100.000000 99.999969 100.000015 ...
        99.999947 100.000061 53.047077 110.000046 117.991989 65.000000 ...
        68.760147 69.999931 71.428818 75.000038 76.843384 80.968781 ...
        88.646126 63.864388 68.155350 72.547775 75.584831 58.379192 ...
        80.950836 64.135651 54.384785 73.821884 64.437073 59.176456 ...
        65.521278 61.399822 58.144047 57.004543 64.126297];
    nbands = [1 1 1 1 1 1 1 1 2 1 1 1 1 1 2 1 1 2 2 2 2 2 2 2 2 3 3 3 3 4 ...
        3 4 5 4 5 6 6 7 8 9 9];

    if fs == 16000
        C.DS = 64;  C.AlignNfft = 1024;
        C.Nb = 49;  C.Sl = 1.866055e-1;  C.Sp = 6.910853e-6;
        C.nbands = [nbands 12 12 15 16 18 21 25 20];
        C.pdc    = [pdc 54.311001 61.114979 55.077751 56.849335 55.628868 ...
                    53.137054 54.985844 79.546974];
        C.InIIR = [0.325631521 -0.086782860 -0.238848661 -1.079416490 0.434583902; ...
                   0.403961804 -0.556985881  0.153024077 -0.415115835 0.696590244; ...
                   4.736162769  3.287251046  1.753289019 -1.859599046 0.876284034; ...
                   0.365373469  0.000000000  0.000000000 -0.634626531 0.000000000; ...
                   0.884811506  0.000000000  0.000000000 -0.256725271 0.141536777; ...
                   0.723593055 -1.447186099  0.723593044 -1.129587469 0.657232737; ...
                   1.644910855 -1.817280902  1.249658063 -1.778403899 0.801724355; ...
                   0.633692689 -0.284644314 -0.319789663  0.000000000 0.000000000; ...
                   1.032763031  0.268428979  0.602913323  0.000000000 0.000000000; ...
                   1.001616361 -0.823749013  0.439731942 -0.885778255 0.000000000; ...
                   0.752472096 -0.375388990  0.188977609 -0.077258216 0.247230734; ...
                   1.023700575  0.001661628  0.521284240 -0.183867259 0.354324187];
        C.WBIIR = [2.740826 -5.4816519 2.740826 -1.9444777 0.94597794];
        C.abs_thresh  = abs_thresh(1:49);
        C.bark_centre = bark_centre(1:49);
        C.bark_width  = bark_width(1:49);
        C.pdc = C.pdc(1:49);
    elseif fs == 8000
        C.DS = 32;  C.AlignNfft = 512;
        C.Nb = 42;  C.Sl = 1.866055e-1;  C.Sp = 2.764344e-5;
        C.nbands = [nbands 11];
        C.pdc    = [pdc 59.248363];
        C.InIIR = [0.885535424 -0.885535424  0.000000000 -0.771070709 0.000000000; ...
                   0.895092588  1.292907193  0.449260174  1.268869037 0.442025372; ...
                   4.049527940 -7.865190042  3.815662102 -1.746859852 0.786305963; ...
                   0.500002353 -0.500002353  0.000000000  0.000000000 0.000000000; ...
                   0.565002834 -0.241585934 -0.306009671  0.259688659 0.249979657; ...
                   2.115237288  0.919935084  1.141240051 -1.587313419 0.665935315; ...
                   0.912224584 -0.224397719 -0.641121413 -0.246029464 -0.556720590; ...
                   0.444617727 -0.307589321  0.141638062 -0.996391149 0.502251622];
        C.WBIIR = [2.6657628 -5.3315255 2.6657628 -1.8890331 0.89487434];
        C.abs_thresh  = abs_thresh(1:42);
        C.bark_centre = bark_centre(1:42);
        C.bark_width  = bark_width(1:42);
    else
        error('pesq_score: fs must be 8000 or 16000.');
    end
    C.Nf = 8 * C.DS;                % psychoacoustic frame length
    % Hz-bin -> Bark-band summation matrix (includes power density correction)
    C.W = zeros(C.Nb, C.Nf/2);
    k = 0;
    for b = 1:C.Nb
        C.W(b, k+1:k+C.nbands(b)) = C.pdc(b) * C.Sp;
        k = k + C.nbands(b);
    end
end

% =========================================================================
%  Pre-processing
% =========================================================================
function s = load_src(x, C)
    pad = C.SB * C.DS;
    s.Nsamples = numel(x) + 2 * pad;
    s.data = [zeros(pad, 1); x; zeros(C.DPAD + pad, 1)];
end

function p = nextpow2_c(x)
    p = 1;
    while p < x
        p = p * 2;
    end
end

function y = real_ifft_half(Xh, P)
    y = real(ifft([Xh; conj(Xh(P/2:-1:2))]));
end

function data = apply_filter(data, nsamp, curve, C)
% FFT-domain filtering with a piecewise-linear dB magnitude curve.
    ofs = C.SB * C.DS;
    n = nsamp - 2 * ofs + C.DPAD;
    P = nextpow2_c(n);
    x = zeros(P, 1);
    x(1:n) = data(ofs+1:ofs+n);
    X = fft(x);
    f = (0:P/2)' * (C.Fs / P);
    g = interp1(curve(:,1), curve(:,2), f, 'linear', 'extrap') ...
        - interp1(curve(:,1), curve(:,2), 1000, 'linear', 'extrap');
    Xh = X(1:P/2+1) .* 10.^(g / 20);
    y = real_ifft_half(Xh, P);
    data(ofs+1:ofs+n) = y(1:n);
end

function s = fix_power_level(s, maxN, C)
    ofs = C.SB * C.DS;
    af = apply_filter(s.data, s.Nsamples, C.align_dB, C);
    pw = sum(af(ofs+1 : s.Nsamples - ofs + C.DPAD).^2) / (maxN - 2*ofs + C.DPAD);
    g = sqrt(1e7 / pw);
    s.data(1:s.Nsamples) = s.data(1:s.Nsamples) * g;
end

function data = wb_input_filter(data, nsamp, C)
    ofs = C.SB * C.DS;
    for i = 0:15
        data(ofs + i) = data(ofs + i) * i / 16;               % C: ofs+i-1
        data(nsamp - ofs - i + 1) = data(nsamp - ofs - i + 1) * i / 16;
    end
    idx = ofs+1 : nsamp - ofs;
    data(idx) = sos_filter(C.WBIIR, data(idx));
end

function y = sos_filter(H, x)
    y = x;
    for k = 1:size(H, 1)
        y = filter(H(k, 1:3), [1 H(k, 4:5)], y);
    end
end

function data = dc_block(data, nsamp, C)
    ofs = C.SB * C.DS;
    idx = ofs+1 : nsamp - ofs;
    data(idx) = data(idx) - sum(data(idx)) / nsamp;
    for c = 0:C.DS-1
        data(ofs + c + 1) = data(ofs + c + 1) * (0.5 + c) / C.DS;
    end
    for c = 0:C.DS-1
        data(nsamp - ofs - c) = data(nsamp - ofs - c) * (0.5 + c) / C.DS;
    end
end

function [VAD, logVAD] = apply_vad(data, nsamp, C)
% Energy-based voice activity detector of P.862 (4 ms windows).
    Nw  = fix(nsamp / C.DS);
    VAD = mean(reshape(data(1:Nw*C.DS).^2, C.DS, Nw), 1)';
    LevelThresh = mean(VAD);
    LevelMin = max(VAD);
    if LevelMin > 0, LevelMin = LevelMin * 1e-4; else, LevelMin = 1; end
    VAD(VAD < LevelMin) = LevelMin;
    for it = 1:12
        sel = VAD <= LevelThresh;
        LevelNoise = 0; StDNoise = 0;
        if any(sel)
            LevelNoise = mean(VAD(sel));
            StDNoise = sqrt(mean((VAD(sel) - LevelNoise).^2));
        end
        LevelThresh = 1.001 * (LevelNoise + 2 * StDNoise);
    end
    sp = VAD > LevelThresh;
    if any(sp), LevelSig = mean(VAD(sp)); else, LevelSig = 0; LevelThresh = -1; end
    if any(~sp), LevelNoise = mean(VAD(~sp)); else, LevelNoise = 1; end
    VAD(VAD <= LevelThresh) = -VAD(VAD <= LevelThresh);
    VAD(1)  = -LevelMin;
    VAD(Nw) = -LevelMin;

    % remove very short speech bursts
    start = 0;
    for c = 1:Nw-1                                   % C index c
        if VAD(c+1) > 0 && VAD(c) <= 0, start = c; end
        if VAD(c+1) <= 0 && VAD(c) > 0
            finish = c;
            if finish - start <= C.MINSPEECHLGTH
                VAD(start+1:finish) = -VAD(start+1:finish);
            end
        end
    end
    % remove low-energy bursts if the signal is very clean
    if LevelSig >= LevelNoise * 1000
        start = 0;
        for c = 1:Nw-1
            if VAD(c+1) > 0 && VAD(c) <= 0, start = c; end
            if VAD(c+1) <= 0 && VAD(c) > 0
                finish = c;
                g = sum(VAD(start+1:finish));
                if g < 3 * LevelThresh * (finish - start)
                    VAD(start+1:finish) = -VAD(start+1:finish);
                end
            end
        end
    end
    % join speech segments separated by short pauses
    start = 0; finish = 0;
    for c = 1:Nw-1
        if VAD(c+1) > 0 && VAD(c) <= 0
            start = c;
            if finish > 0 && (start - finish) <= C.JOINSPEECHLGTH
                VAD(finish+1:start) = LevelMin;
            end
        end
        if VAD(c+1) <= 0 && VAD(c) > 0, finish = c; end
    end
    start = 0;
    for c = 1:Nw-1
        if VAD(c+1) > 0 && VAD(c) <= 0, start = c; end
    end
    if start == 0
        VAD = abs(VAD);
        VAD(1) = -LevelMin;
        VAD(Nw) = -LevelMin;
    end
    % soften onsets / offsets
    c = 3;
    while c < Nw - 2
        if VAD(c+1) > 0 && VAD(c-1) <= 0
            VAD(c-1) = VAD(c+1) * 0.1;
            VAD(c)   = VAD(c+1) * 0.3;
            c = c + 1;
        end
        if VAD(c+1) <= 0 && VAD(c) > 0
            VAD(c+1) = VAD(c) * 0.3;
            VAD(c+2) = VAD(c) * 0.1;
            c = c + 3;
        end
        c = c + 1;
    end
    VAD(VAD < 0) = 0;
    if LevelThresh <= 0, LevelThresh = LevelMin; end
    logVAD = zeros(Nw, 1);
    act = VAD > LevelThresh;
    logVAD(act) = log(VAD(act) / LevelThresh);
end

% =========================================================================
%  Time alignment
% =========================================================================
function dly = crude_delay(rv, nr, sr, dv, nd, sd)
% Envelope cross-correlation (logVAD) -> delay in VAD windows.
    Imax = nr - 1;
    if nr > 1 && nd > 1
        Y = conv(flipud(rv(sr+1:sr+nr)), dv(sd+1:sd+nd));
        m = 0;
        for c = 0:nr+nd-2
            if Y(c+1) > m
                m = Y(c+1);
                Imax = c;
            end
        end
    end
    dly = Imax - nr + 1;
end

function E = crude_align_utt(r, d, E, uid, C)
    startr = E.UttSearch_Start(uid);
    startd = startr + fix(E.Crude_DelayEst / C.DS);
    if startd < 0
        startr = -fix(E.Crude_DelayEst / C.DS);
        startd = 0;
    end
    nr = E.UttSearch_End(uid) - startr;
    nd = nr;
    if startd + nd > fix(d.Nsamples / C.DS)
        nd = fix(d.Nsamples / C.DS) - startd;
    end
    E.Utt_DelayEst(uid) = crude_delay(r.logVAD, nr, startr, d.logVAD, nd, startd) ...
                          * C.DS + E.Crude_DelayEst;
end

function dly = crude_align_test(r, d, estdelay, sstart, send, C)
% crude_align() with Utt_id == MAXNUTTERANCES (used by split_align).
    startr = sstart;
    startd = startr + fix(estdelay / C.DS);
    if startd < 0
        startr = -fix(estdelay / C.DS);
        startd = 0;
    end
    nr = send - startr;
    nd = nr;
    if startd + nd > fix(d.Nsamples / C.DS)
        nd = fix(d.Nsamples / C.DS) - startd;
    end
    dly = crude_delay(r.logVAD, nr, startr, d.logVAD, nd, startd) * C.DS + estdelay;
end

function w = hann_periodic(N)
    w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
end

function [pk, wt] = xcorr_peaks(r, d, starts_r, starts_d, C)
% For each frame pair: indicator of lags within 99 % of the max |xcorr|
% and the frame weight (0.99*max)^0.125.
    N = C.AlignNfft;
    nf = numel(starts_r);
    pk = false(N, nf);
    wt = zeros(1, nf);
    if nf == 0, return; end
    w = hann_periodic(N);
    idx = (0:N-1)';
    X1 = fft(r.data(idx + starts_r(:)' + 1) .* repmat(w, 1, nf));
    X2 = fft(d.data(idx + starts_d(:)' + 1) .* repmat(w, 1, nf));
    xc = abs(real(ifft(conj(X1) .* X2)));
    vmax = 0.99 * max(xc, [], 1);
    pk = xc > repmat(vmax, N, 1);
    wt = vmax .^ 0.125;
end

function h = circ_tri_smooth(v, kernel)
% circular convolution with triangle 1 - |k|/kernel, |k| < kernel
    h = v;
    for k = 1:kernel-1
        h = h + (1 - k/kernel) * (circshift(v, k) + circshift(v, -k));
    end
end

function E = time_align(r, d, E, uid, C)
    N = C.AlignNfft;
    estdelay = E.Utt_DelayEst(uid);
    startr = E.UttSearch_Start(uid) * C.DS;
    startd = startr + estdelay;
    if startd < 0
        startr = -estdelay;
        startd = 0;
    end
    nf = 0;
    while (startd + (nf)*N/4 + N <= d.Nsamples) && ...
          (startr + (nf)*N/4 + N <= E.UttSearch_End(uid) * C.DS)
        nf = nf + 1;
    end
    sr = startr + (0:nf-1) * N/4;
    sd = startd + (0:nf-1) * N/4;
    [pk, wt] = xcorr_peaks(r, d, sr, sd, C);
    H = double(pk) * wt(:);
    Hsum = sum(H);
    H = abs(circ_tri_smooth(H, N/64));
    if Hsum > 0, H = H / Hsum; else, H = zeros(N, 1); end
    [vmax, I] = max(H);
    Imax = I - 1;
    if vmax <= 0, Imax = 0; vmax = 0; end
    if Imax >= N/2, Imax = Imax - N; end
    E.Utt_Delay(uid) = estdelay + Imax;
    E.Utt_DelayConf(uid) = vmax;
end

function E = id_searchwindows(r, d, E, C)
    VL = fix(r.Nsamples / C.DS);
    dstart = C.MINUTTLENGTH - fix(E.Crude_DelayEst / C.DS);
    dend = fix((d.Nsamples - E.Crude_DelayEst) / C.DS) - C.MINUTTLENGTH;
    n = 0; flag = 0; this_start = 0;
    E.UttSearch_Start = zeros(1, C.MAXNUTT);
    E.UttSearch_End = zeros(1, C.MAXNUTT);
    for c = 0:VL-1
        v = r.VAD(c+1);
        if v > 0 && flag == 0
            flag = 1;
            this_start = c;
            E.UttSearch_Start(n+1) = max(c - C.SB, 0);
        end
        if (v == 0 || c == VL-1) && flag == 1
            flag = 0;
            E.UttSearch_End(n+1) = min(c + C.SB, VL - 1);
            if (c - this_start) >= C.MINUTTLENGTH && this_start < dend && c > dstart
                n = n + 1;
            end
        end
    end
    E.Nutterances = n;
end

function E = id_utterances(r, d, E, C)
    VL = fix(r.Nsamples / C.DS);
    dstart = C.MINUTTLENGTH - fix(E.Crude_DelayEst / C.DS);
    dend = fix((d.Nsamples - E.Crude_DelayEst) / C.DS) - C.MINUTTLENGTH;
    n = 0; flag = 0; this_start = 0;
    for c = 0:VL-1
        v = r.VAD(c+1);
        if v > 0 && flag == 0
            flag = 1;
            this_start = c;
            E.Utt_Start(n+1) = c;
        end
        if (v == 0 || c == VL-1) && flag == 1
            flag = 0;
            E.Utt_End(n+1) = c;
            if (c - this_start) >= C.MINUTTLENGTH && this_start < dend && c > dstart
                n = n + 1;
            end
        end
    end
    Nu = E.Nutterances;
    E.Utt_Start(1) = C.SB;
    E.Utt_End(Nu) = VL - C.SB;
    for u = 2:Nu
        c = fix((E.Utt_Start(u) + E.Utt_End(u-1)) / 2);
        E.Utt_Start(u) = c;
        E.Utt_End(u-1) = c;
    end
    this_start = E.Utt_Start(1) * C.DS + E.Utt_Delay(1);
    if this_start < C.SB * C.DS
        E.Utt_Start(1) = C.SB + fix((C.DS - 1 - E.Utt_Delay(1)) / C.DS);
    end
    last_end = E.Utt_End(Nu) * C.DS + E.Utt_Delay(Nu);
    if last_end > d.Nsamples - C.SB * C.DS
        E.Utt_End(Nu) = fix((d.Nsamples - E.Utt_Delay(Nu)) / C.DS) - C.SB;
    end
    for u = 2:Nu
        this_start = E.Utt_Start(u) * C.DS + E.Utt_Delay(u);
        last_end = E.Utt_End(u-1) * C.DS + E.Utt_Delay(u-1);
        if this_start < last_end
            c = fix((this_start + last_end) / 2);
            E.Utt_Start(u) = fix((C.DS - 1 + c - E.Utt_Delay(u)) / C.DS);
            E.Utt_End(u-1) = fix((c - E.Utt_Delay(u-1)) / C.DS);
        end
    end
end

function E = utterance_locate(r, d, E, C)
    E = id_searchwindows(r, d, E, C);
    Z = zeros(1, C.MAXNUTT);
    E.Utt_DelayEst = Z; E.Utt_Delay = Z; E.Utt_DelayConf = Z;
    E.Utt_Start = Z; E.Utt_End = Z;
    for u = 1:E.Nutterances
        E = crude_align_utt(r, d, E, u, C);
        E = time_align(r, d, E, u, C);
    end
    if E.Nutterances < 1, return; end
    E = id_utterances(r, d, E, C);
    E = utterance_split(r, d, E, C);
end

function E = utterance_split(r, d, E, C)
    u = 1;
    while u <= E.Nutterances && E.Nutterances < C.MAXNUTT
        dEst  = E.Utt_DelayEst(u);
        dConf = E.Utt_DelayConf(u);
        uS = E.Utt_Start(u);
        uE = E.Utt_End(u);
        sS = uS;
        while sS < uE && r.VAD(sS+1) <= 0, sS = sS + 1; end
        sE = uE;
        while sE > uS && r.VAD(sE+1) <= 0, sE = sE - 1; end
        sE = sE + 1;
        if (sE - sS) >= 200
            [ED1, D1, DC1, ED2, D2, DC2, BP] = split_align(r, d, uS, sS, sE, uE, ...
                                                           dEst, dConf, C);
            if DC1 > dConf && DC2 > dConf
                for st = E.Nutterances:-1:u+1
                    E.Utt_DelayEst(st+1)  = E.Utt_DelayEst(st);
                    E.Utt_Delay(st+1)     = E.Utt_Delay(st);
                    E.Utt_DelayConf(st+1) = E.Utt_DelayConf(st);
                    E.Utt_Start(st+1)     = E.Utt_Start(st);
                    E.Utt_End(st+1)       = E.Utt_End(st);
                    E.UttSearch_Start(st+1) = E.Utt_Start(st);
                    E.UttSearch_End(st+1)   = E.Utt_End(st);
                end
                E.Nutterances = E.Nutterances + 1;
                E.Utt_DelayEst(u) = ED1;  E.Utt_Delay(u) = D1;  E.Utt_DelayConf(u) = DC1;
                E.Utt_DelayEst(u+1) = ED2; E.Utt_Delay(u+1) = D2; E.Utt_DelayConf(u+1) = DC2;
                E.UttSearch_Start(u+1) = E.UttSearch_Start(u);
                E.UttSearch_End(u+1) = E.UttSearch_End(u);
                if D2 < D1
                    E.Utt_Start(u) = uS;     E.Utt_End(u) = BP;
                    E.Utt_Start(u+1) = BP;   E.Utt_End(u+1) = uE;
                else
                    E.Utt_Start(u) = uS;
                    E.Utt_End(u) = BP + fix((D2 - D1) / (2 * C.DS));
                    E.Utt_Start(u+1) = BP - fix((D2 - D1) / (2 * C.DS));
                    E.Utt_End(u+1) = uE;
                end
                if (E.Utt_Start(u) - C.SB) * C.DS + D1 < 0
                    E.Utt_Start(u) = C.SB + fix((C.DS - 1 - D1) / C.DS);
                end
                if E.Utt_End(u+1) * C.DS + D2 > d.Nsamples - C.SB * C.DS
                    E.Utt_End(u+1) = fix((d.Nsamples - D2) / C.DS) - C.SB;
                end
            else
                u = u + 1;
            end
        else
            u = u + 1;
        end
    end
end

function [bED1, bD1, bDC1, bED2, bD2, bDC2, bBP] = split_align(r, d, uS, sS, sE, uE, dEst, dConf, C)
    N = C.AlignNfft;
    kernel = N / 64;
    bED1 = 0; bD1 = 0; bDC1 = 0; bED2 = 0; bD2 = 0; bDC2 = 0; bBP = 0;
    uLen = sE - sS;
    Delta = N / (4 * C.DS);
    Step = fix((0.801 * uLen + 40 * Delta - 1) / (40 * Delta)) * Delta;
    Pad = fix(uLen / 10);
    if Pad < 75, Pad = 75; end
    BPs = sS + Pad;
    nb = 0;
    while true
        nb = nb + 1;
        BPs(nb+1) = BPs(nb) + Step;
        if ~((BPs(nb+1) <= (sE - Pad)) && (nb < 40)), break; end
    end
    if nb <= 0, return; end
    BPs = BPs(1:nb);
    ED1 = zeros(1, nb); ED2 = zeros(1, nb);
    for b = 1:nb
        ED1(b) = crude_align_test(r, d, dEst, uS, BPs(b), C);
        ED2(b) = crude_align_test(r, d, dEst, BPs(b), uE, C);
    end

    % ---- forward (first part) --------------------------------------------
    D1 = zeros(1, nb); DC1 = -2 * ones(1, nb);
    for est = unique(ED1)
        startr = uS * C.DS;
        startd = startr + est;
        if startd < 0, startr = -est; startd = 0; end
        maxbp = max(BPs(ED1 == est));
        nf = 0;
        while (startd + nf*N/4 + N <= d.Nsamples) && (startr + nf*N/4 + N <= maxbp * C.DS)
            nf = nf + 1;
        end
        sr = startr + (0:nf-1) * N/4;
        [pk, wt] = xcorr_peaks(r, d, sr, startd + (0:nf-1) * N/4, C);
        for b = find(ED1 == est)
            use = (sr + N) <= BPs(b) * C.DS;
            [D1(b), DC1(b)] = hist_peak(pk(:, use), wt(use), kernel, est, N);
        end
    end
    % ---- backward (second part) ------------------------------------------
    D2 = zeros(1, nb);
    DC2 = zeros(1, nb);
    DC2(DC1 > dConf) = -2;
    todo = find(DC2 <= -2);
    for est = unique(ED2(todo))
        startr = uE * C.DS - N;
        startd = startr + est;
        if startd + N > d.Nsamples
            startd = d.Nsamples - N;
            startr = startd - est;
        end
        minbp = min(BPs(todo(ED2(todo) == est)));
        nf = 0;
        while (startd - nf*N/4 >= 0) && (startr - nf*N/4 >= minbp * C.DS)
            nf = nf + 1;
        end
        sr = startr - (0:nf-1) * N/4;
        [pk, wt] = xcorr_peaks(r, d, sr, startd - (0:nf-1) * N/4, C);
        for b = todo(ED2(todo) == est)
            use = sr >= BPs(b) * C.DS;
            [D2(b), DC2(b)] = hist_peak(pk(:, use), wt(use), kernel, est, N);
        end
    end
    for b = 1:nb
        if abs(D2(b) - D1(b)) >= C.DS && (DC1(b) + DC2(b)) > (bDC1 + bDC2) && ...
                DC1(b) > dConf && DC2(b) > dConf
            bED1 = ED1(b); bD1 = D1(b); bDC1 = DC1(b);
            bED2 = ED2(b); bD2 = D2(b); bDC2 = DC2(b);
            bBP = BPs(b);
        end
    end
end

function [D, DC] = hist_peak(pk, wt, kernel, est, N)
    A = double(pk) * (wt(:) / kernel);
    Hsum = sum(A) * kernel;
    H = circ_tri_smooth(A, kernel) * kernel;
    [vmax, I] = max(H);
    Imax = I - 1;
    if vmax <= 0, Imax = 0; vmax = 0; end
    if Imax >= N/2, Imax = Imax - N; end
    D = est + Imax;
    if Hsum > 0, DC = vmax / Hsum; else, DC = 0; end
end

% =========================================================================
%  Perceptual model
% =========================================================================
function P = bark_spectra(data, starts, C)
    Nf = C.Nf;
    nf = numel(starts);
    P = zeros(C.Nb, nf);
    if nf == 0, return; end
    idx = (0:Nf-1)';
    X = fft(data(idx + starts(:)' + 1) .* repmat(hann_periodic(Nf), 1, nf));
    S = abs(X(1:Nf/2, :)).^2;
    S(1, :) = 0;
    P = C.W * S;
end

function L = intensity_warping(ppd, C)
    h = ones(C.Nb, 1);
    lo = C.bark_centre(:) < 4;
    h(lo) = 6 ./ (C.bark_centre(lo)' + 2);
    h(h > 2) = 2;
    zp = 0.23 * h.^0.15;
    th = repmat(C.abs_thresh(:), 1, size(ppd, 2));
    zpm = repmat(zp, 1, size(ppd, 2));
    L = ((th / 0.5).^zpm) .* ((0.5 + 0.5 * ppd ./ th).^zpm - 1);
    L(ppd <= th) = 0;
    L = L * C.Sl;
end

function v = pseudo_lp(x, p, C)
    w = repmat(C.bark_width(2:end)', 1, size(x, 2));
    tw = sum(C.bark_width(2:end));
    v = (sum((abs(x(2:end, :)) .* w).^p, 1) / tw).^(1/p) * tw;
end

function [fd, fda] = frame_disturbances(pr, pd, C, oldScale)
% pr, pd : Bark power densities (ref already freq-compensated) for a run of
% consecutive frames.  Applies the gain compensation, loudness transform,
% dead zone and asymmetry factor; returns per-frame disturbances.
    nf = size(pr, 2);
    th = repmat(C.abs_thresh(:), 1, nf);
    tar = sum(pr(2:end, :) .* (pr(2:end, :) > th(2:end, :)), 1);
    tad = sum(pd(2:end, :) .* (pd(2:end, :) > th(2:end, :)), 1);
    for f = 1:nf
        scale = (tar(f) + 5e3) / (tad(f) + 5e3);
        if ~isempty(oldScale)
            scale = 0.2 * oldScale + 0.8 * scale;
        end
        oldScale = scale;
        scale = min(max(scale, 3e-4), 5);
        pd(:, f) = pd(:, f) * scale;
    end
    Lr = intensity_warping(pr, C);
    Ld = intensity_warping(pd, C);
    D = Ld - Lr;
    m = 0.25 * min(Ld, Lr);
    D = (D - m) .* (D > m) + (D + m) .* (D < -m);
    fd = pseudo_lp(D, 2, C);
    ratio = ((pd + 50) ./ (pr + 50)).^1.2;
    ratio(ratio > 12) = 12;
    ratio(ratio < 3) = 0;
    fda = pseudo_lp(D .* ratio, 1, C);
end

function raw = psychoacoustic_model(r, d, E, C)
    SBD  = C.SB * C.DS;
    maxN = max(r.Nsamples, d.Nsamples);
    Nf   = C.Nf;
    hop  = Nf / 2;
    crit = 500;

    % skip leading / trailing digital silence of the reference
    half = fix(maxN / 2);
    a = abs(r.data);
    s5 = filter(ones(5, 1), 1, a);                  % s5(k) = sum a(k-4..k)
    pos = SBD + (0:half) ;                           % C start index
    v = s5(pos + 5);
    k = find(v >= crit, 1);
    if isempty(k), skip_start = half; else, skip_start = min(k - 1, half); end
    base = maxN - SBD + C.DPAD - 1;                  % C index of last sample
    pos = base - (0:half);
    v = s5(pos + 1);                                 % sum a(pos-4 .. pos)
    k = find(v >= crit, 1);
    if isempty(k), skip_end = half; else, skip_end = min(k - 1, half); end

    start_frame = fix(skip_start / hop);
    stop_frame  = fix((maxN - 2*SBD + C.DPAD - skip_end) / hop) - 1;
    nfr = stop_frame + 1;

    % per-frame delay from the utterance table
    Nu = E.Nutterances;
    starts_ref = SBD + (0:stop_frame) * hop;
    starts_deg = starts_ref + utt_delay_at(starts_ref, E, C);

    ppd_ref = bark_spectra(r.data, starts_ref, C);
    ok = (starts_deg > 0) & (starts_deg + Nf < maxN + C.DPAD);
    ppd_deg = zeros(C.Nb, nfr);
    ppd_deg(:, ok) = bark_spectra(d.data, starts_deg(ok), C);

    th = repmat(C.abs_thresh(:), 1, nfr);
    tot_ref100 = sum(ppd_ref(2:end, :) .* (ppd_ref(2:end, :) > 100 * th(2:end, :)), 1);
    silent = tot_ref100 < 1e7;

    % frequency response compensation of the reference
    ntot = fix((maxN - 2*SBD + C.DPAD) / hop) - 1;
    avg_ref = sum(ppd_ref(:, ~silent) .* (ppd_ref(:, ~silent) > 100 * th(:, ~silent)), 2) / ntot;
    avg_deg = sum(ppd_deg(:, ~silent) .* (ppd_deg(:, ~silent) > 100 * th(:, ~silent)), 2) / ntot;
    x = (avg_deg + 1000) ./ (avg_ref + 1000);
    x(x > 100) = 100;
    x(x < 0.01) = 0.01;
    ppd_ref = ppd_ref .* repmat(x, 1, nfr);

    total_power_ref = sum(ppd_ref(2:end, :) .* (ppd_ref(2:end, :) > th(2:end, :)), 1);
    [fd, fda] = frame_disturbances(ppd_ref, ppd_deg, C, []);
    there_is_bad = any(fd > 30);

    % frames skipped because of a large negative delay jump
    for u = 2:Nu
        frame1 = fix(((E.Utt_Start(u) - C.SB) * C.DS + E.Utt_Delay(u)) / hop);
        j = fix(((E.Utt_End(u-1) - C.SB) * C.DS + E.Utt_Delay(u-1)) / hop);
        jump = E.Utt_Delay(u) - E.Utt_Delay(u-1);
        if frame1 > j, frame1 = j; end
        if frame1 < 0, frame1 = 0; end
        if jump < -hop
            frame2 = fix(((E.Utt_Start(u) - C.SB) * C.DS + max(0, abs(jump))) / hop) + 1;
            for f = frame1:frame2
                if f < stop_frame
                    fd(f+1) = 0;
                    fda(f+1) = 0;
                end
            end
        end
    end

    if there_is_bad
        % degraded signal with per-utterance delays applied
        nn = C.DPAD + maxN;
        tweaked = zeros(nn, 1);
        i = (SBD:nn-SBD-1);
        j = i + utt_delay_at(i, E, C);
        j(j < SBD) = SBD;
        j(j >= nn - SBD) = nn - SBD - 1;
        tweaked(i + 1) = d.data(j + 1);

        bad = fd > 30;
        bad(1) = false;
        smeared = false(1, nfr);
        for f = 2:stop_frame-3
            smeared(f+1) = min(max(bad(f-1:f+1)), max(bad(f+1:f+3)));
        end
        % bad intervals
        bstart = []; bstop = [];
        f = 0;
        while f <= stop_frame
            while f <= stop_frame && ~smeared(f+1), f = f + 1; end
            if f <= stop_frame
                s0 = f;
                while f <= stop_frame && smeared(f+1), f = f + 1; end
                if f <= stop_frame
                    if f - s0 >= 5
                        bstart(end+1) = s0; %#ok<AGROW>
                        bstop(end+1) = f;   %#ok<AGROW>
                    end
                end
            end
        end
        if ~isempty(bstart)
            srange = 4 * Nf;
            dbl = tweaked;
            for b = 1:numel(bstart)
                ss = bstart(b) * hop + SBD;
                se = bstop(b) * hop + Nf + SBD;
                ns = se - ss;
                refseg = [zeros(srange, 1); r.data(ss+1:ss+ns); zeros(srange, 1)];
                jj = ss - srange + (0:2*srange+ns-1)';
                lim = maxN - SBD + C.DPAD;
                jj(jj < SBD) = SBD;
                jj(jj >= lim) = lim - 1;
                degseg = tweaked(jj + 1);
                [dly, corr] = compute_delay(refseg, degseg, srange);
                if corr < 0.5, dly = 0; end
                ii = (ss:se-1)';
                jj = ii + dly;
                jj(jj < 0) = 0;
                jj(jj >= maxN) = maxN - 1;
                dbl(ii + 1) = tweaked(jj + 1);
            end
            for b = 1:numel(bstart)
                fr = bstart(b):bstop(b)-1;
                pdb = bark_spectra(dbl, SBD + fr * hop, C);
                [fd2, fda2] = frame_disturbances(ppd_ref(:, fr+1), pdb, C, 1);
                fd(fr+1) = min(fd(fr+1), fd2);
                fda(fr+1) = min(fda(fr+1), fda2);
            end
        end
    end

    % time weighting (long files) and aggregation
    tw = ones(1, nfr);
    if nfr > 1000
        n = fix((maxN - 2*SBD) / hop) - 1;
        twf = min((n - 1000) / 5500, 0.5);
        tw = (1 - twf) + twf * (0:stop_frame) / n;
    end
    h = ((total_power_ref + 1e5) / 1e7).^0.04;
    fd = min(fd ./ h, 45);
    fda = min(fda ./ h, 45);
    d_ind = lpq_weight(start_frame, stop_frame, 6, 2, fd, tw);
    a_ind = lpq_weight(start_frame, stop_frame, 6, 2, fda, tw);
    raw = 4.5 - 0.1 * d_ind - 0.0309 * a_ind;
end

function dl = utt_delay_at(pos, E, C)
% Delay of the last utterance whose start is <= pos (utterance 1 if none).
    u = zeros(size(pos));
    for k = 1:E.Nutterances
        u(pos >= E.Utt_Start(k) * C.DS) = k;
    end
    u(u == 0) = 1;
    dl = E.Utt_Delay(u);
    dl = reshape(dl, size(pos));
end

function [best, maxc] = compute_delay(ts1, ts2, srange)
    n = numel(ts1);
    P = nextpow2_c(2 * n);
    p1 = sum(ts1.^2) / P;
    p2 = sum(ts2.^2) / P;
    best = 0; maxc = 0;
    if p1 <= 1e-6 || p2 <= 1e-6, return; end
    nrm = sqrt(p1 * p2);
    x1 = zeros(P, 1); x2 = zeros(P, 1);
    x1(1:n) = abs(ts1); x2(1:n) = abs(ts2);
    y = real(ifft(conj(fft(x1) / P) .* fft(x2)));
    lags = [(-srange:-1) + P, 0:srange-1];
    vals = abs(y(lags + 1)) / nrm;
    shifts = [-srange:-1, 0:srange-1];
    [maxc, k] = max(vals);
    best = shifts(k);
end

function v = lpq_weight(f0, f1, ps, pt, fdist, tw)
    rt = 0; tot = 0;
    for s = f0:10:f1
        fr = s:min(s+19, f1);
        rs = (sum(fdist(fr+1).^ps) / 20)^(1/ps);
        rt = rt + (tw(s - f0 + 1) * rs)^pt;
        tot = tot + tw(s - f0 + 1)^pt;
    end
    v = (rt / tot)^(1/pt);
end
