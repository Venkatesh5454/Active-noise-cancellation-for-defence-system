function n = defence_noise(type, dur, fs, seed)
%DEFENCE_NOISE  Synthesise defence / battlefield noise for dataset generation.
%
%   n = defence_noise(type, dur, fs)          dur in seconds
%   n = defence_noise(type, dur, fs, seed)    reproducible random seed
%
%   type (string):
%     impulsive      : 'gunshot' (single rifle shots), 'machinegun' (burst
%                      fire, ~9-12 rounds/s), 'artillery' (explosions)
%     non-stationary : 'helicopter' (blade slap + rotor harmonics),
%                      'missile' (rising turbine whine + rocket roar),
%                      'drone' (UAV rotor tones), 'siren' (wail / yelp)
%     stationary     : 'vehicle' (armoured-vehicle engine + track clatter),
%                      'hum' (100 Hz generator hum), 'wind', 'white', 'pink'
%     mixture        : 'battlefield' - machine-gun bursts + gunshots +
%                      missile whine + engine hum (like the sample
%                      recordings supplied with the problem statement)
%
%   The output is a column vector normalised to unit RMS, so it can be mixed
%   with speech at a chosen SNR (see mix_at_snr.m).  All components are
%   generated procedurally (no toolbox, no external files) and were
%   calibrated against the recordings in audio/noisy/ (e.g. the machine-gun
%   bursts are ~50-90 ms long with ~10 dB burst-to-gap modulation).

    if nargin < 4 || isempty(seed), seed = 0; end
    rng_state = rng_push(seed);
    N = round(dur * fs);
    t = (0:N-1)' / fs;

    switch lower(type)
        case 'white'
            n = randn(N, 1);
        case 'pink'
            n = pink_noise(N);
        case 'wind'
            n = wind_noise(N, fs);
        case 'hum'
            n = zeros(N, 1);
            f0 = 100 * (1 + 0.002 * randn);
            for k = 1:8
                n = n + (0.8^(k-1)) * sin(2*pi*k*f0*t + 2*pi*rand);
            end
            n = n + 0.05 * pink_noise(N) * rms(n);
        case 'gunshot'
            n = 0.02 * pink_noise(N);
            tt = poisson_times(dur, 0.9, 0.35);
            for k = 1:numel(tt)
                n = add_event(n, gunshot_event(fs, 1), tt(k), fs, db2mag(-3 + 6*rand));
            end
        case 'machinegun'
            n = machinegun(N, fs, dur);
        case 'artillery'
            n = 0.05 * brown_noise(N);
            tt = poisson_times(dur, 0.35, 1.2);
            for k = 1:numel(tt)
                n = add_event(n, explosion_event(fs), tt(k), fs, db2mag(-4 + 8*rand));
            end
        case 'helicopter'
            n = helicopter(N, fs, t);
        case 'missile'
            n = missile(N, fs, t, dur);
        case 'drone'
            n = zeros(N, 1);
            for r = 1:4
                fb = 170 + 40 * rand;
                wob = 1 + 0.01 * sin(2*pi*(0.3 + 0.5*rand)*t + 2*pi*rand);
                ph = 2*pi*cumsum(fb * wob) / fs;
                for k = 1:6
                    n = n + (0.7^k) * sin(k * ph + 2*pi*rand);
                end
            end
            n = n + 0.3 * rms(n) * pink_noise(N);
        case 'siren'
            if rand < 0.6
                T = 3.5 + rand;                                   % wail
                f = 650 + 750 * (0.5 - 0.5 * cos(2*pi*t/T));
            else
                T = 0.25 + 0.1*rand;                              % yelp
                f = 650 + 750 * mod(t / T, 1);
            end
            ph = 2*pi*cumsum(f) / fs;
            n = sin(ph) + 0.35*sin(3*ph) + 0.15*sin(5*ph);
            n = n + 0.05 * pink_noise(N);
        case 'vehicle'
            n = vehicle(N, fs, t);
        case 'battlefield'
            n = unit_rms(machinegun(N, fs, dur)) * 1.0 ...
              + unit_rms(gunshot_track(N, fs, dur)) * 0.7 ...
              + unit_rms(missile(N, fs, t, dur)) * 0.35 ...
              + unit_rms(vehicle(N, fs, t)) * 0.3;
        otherwise
            rng_pop(rng_state);
            error('defence_noise: unknown noise type ''%s''.', type);
    end
    n = unit_rms(n(:));
    rng_pop(rng_state);
end

% =========================================================================
function n = machinegun(N, fs, dur)
% Burst fire: 1-3 s bursts separated by 0.3-1 s pauses, 9-12 rounds/s.
    n = 0.01 * pink_noise(N);
    floor_env = zeros(N, 1);
    tcur = 0.1 * rand;
    while tcur < dur
        blen = 1 + 2 * rand;
        rate = 9 + 3 * rand;
        tb = tcur;
        while tb < min(tcur + blen, dur)
            n = add_event(n, gunshot_event(fs, 2), tb, fs, db2mag(-2 + 4*rand));
            tb = tb + (1 / rate) * (1 + 0.08 * randn);
        end
        i0 = max(1, round(tcur*fs) + 1);
        i1 = min(N, round(min(tcur + blen + 0.15, dur) * fs));
        floor_env(i0:i1) = 1;
        tcur = tcur + blen + 0.3 + 0.7 * rand;
    end
    % reverberant floor between rounds (about 10 dB below the bursts)
    sm = filter(1 - 0.999, [1 -0.999], floor_env);
    fl = biquad(brown_noise(N) + 0.3 * pink_noise(N), 'lp', 2500, 0.7, fs);
    n = n + 0.28 * rms(n(floor_env > 0)) * unit_rms(fl) .* sm;
end

function n = gunshot_track(N, fs, dur)
    n = zeros(N, 1);
    tt = poisson_times(dur, 0.6, 0.4);
    for k = 1:numel(tt)
        n = add_event(n, gunshot_event(fs, 1), tt(k), fs, db2mag(-3 + 6*rand));
    end
end

function e = gunshot_event(fs, kind)
% kind 1: single rifle shot (sharp crack + blast + echo)
% kind 2: machine-gun round (longer, reverberant 50-90 ms body)
    L = round(0.35 * fs);
    t = (0:L-1)' / fs;
    w = randn(L, 1);
    if kind == 1
        tau_b = 0.006 + 0.006 * rand;     % muzzle blast
        tau_d = 0.030 + 0.020 * rand;     % body
        tau_r = 0.100 + 0.080 * rand;     % reverb tail
        crack = zeros(L, 1);
        m = round(0.0008 * fs);           % supersonic N-wave (~0.8 ms)
        crack(1:2*m) = [linspace(1, -1, 2*m)'];
        e = 1.5 * crack + w .* exp(-t / tau_b) ...
            + 0.6 * biquad(randn(L, 1), 'lp', 900, 0.7, fs) .* exp(-t / tau_d) * 3 ...
            + 0.15 * biquad(randn(L, 1), 'lp', 2000, 0.7, fs) .* exp(-t / tau_r) * 2;
        % one discrete echo
        d = round((0.04 + 0.08 * rand) * fs);
        e(d+1:end) = e(d+1:end) + 0.3 * e(1:end-d);
    else
        tau_b = 0.012 + 0.008 * rand;
        plateau = 0.045 + 0.035 * rand;
        env = exp(-t / tau_b) + 0.35 * (t < plateau) .* exp(-t / 0.08);
        body = biquad(randn(L, 1), 'lp', 1200, 0.7, fs);
        e = 0.5 * w .* env + 1.2 * unit_rms(body) .* env * rms(w .* env) / max(rms(w .* env), eps);
        e = e + 0.2 * biquad(randn(L, 1), 'lp', 1800, 0.7, fs) .* exp(-t / 0.12);
    end
    a = round(0.0005 * fs) + 1;           % ~0.5 ms attack
    e(1:a) = e(1:a) .* linspace(0, 1, a)';
    e = biquad(e, 'hp', 60, 0.7, fs);
end

function e = explosion_event(fs)
    L = round(2.0 * fs);
    t = (0:L-1)' / fs;
    tau = 0.3 + 0.5 * rand;
    boom = biquad(brown_noise(L), 'lp', 250 + 150*rand, 0.7, fs);
    boom = unit_rms(boom) .* exp(-t / tau);
    crack = randn(L, 1) .* exp(-t / 0.015);
    rumble = biquad(randn(L, 1), 'lp', 600, 0.7, fs) .* exp(-t / (2 * tau)) * 0.3;
    e = 2.5 * boom + 0.8 * crack + rumble;
    a = round(0.002 * fs);
    e(1:a) = e(1:a) .* linspace(0, 1, a)';
end

function n = helicopter(N, fs, t)
    fb = 16 + 6 * rand;                                   % blade-pass (Hz)
    ph = 2*pi*fb*t + 2*pi*rand;
    slap = max(cos(ph), 0).^12;                           % impulsive blade slap
    broadband = biquad(pink_noise(N), 'lp', 1500, 0.7, fs);
    n = unit_rms(broadband) .* (0.4 + 1.6 * slap);
    for k = 1:10                                          % main-rotor harmonics
        n = n + 0.5 * (0.75^k) * sin(k * ph + 2*pi*rand);
    end
    ft = 5.3 * fb;                                        % tail rotor
    for k = 1:4
        n = n + 0.15 * (0.6^k) * sin(2*pi*k*ft*t + 2*pi*rand);
    end
    n = n + 0.03 * sin(2*pi*(5200 + 800*rand)*t);         % turbine whine
    n = n + 0.2 * unit_rms(biquad(randn(N, 1), 'hp', 2000, 0.7, fs)) .* (0.3 + slap);
end

function n = missile(N, fs, t, dur)
% Rising turbine / missile whine (harmonic chirp) plus rocket roar.
    f_start = 550 + 250 * rand;
    f_end = f_start * (1.8 + 0.8 * rand);
    f = f_start + (f_end - f_start) * (t / max(dur, eps));
    f = f .* (1 + 0.003 * sin(2*pi*5*t));
    ph = 2*pi*cumsum(f) / fs + 2*pi*rand;
    tone = sin(ph) + 0.6*sin(2*ph) + 0.45*sin(3*ph) + 0.2*sin(5*ph);
    roar = biquad(pink_noise(N), 'lp', 2500, 0.7, fs);
    grow = 0.6 + 0.4 * t / max(dur, eps);
    n = unit_rms(tone) + 0.7 * unit_rms(roar) .* grow;
end

function n = vehicle(N, fs, t)
    fe = 32 + 12 * rand;                                  % engine firing rate
    ph = 2*pi*fe*t + 2*pi*rand;
    n = zeros(N, 1);
    for k = 1:12
        n = n + (1 / k) * sin(k * ph + 2*pi*rand);        % sawtooth-like
    end
    n = unit_rms(n) + 0.8 * unit_rms(biquad(brown_noise(N), 'lp', 400, 0.7, fs));
    ftr = 12 + 6 * rand;                                  % track-link clatter
    clat = zeros(N, 1);
    tk = 0;
    while tk < t(end)
        i = round(tk * fs) + 1;
        L = min(round(0.02 * fs), N - i + 1);
        if L > 0
            tt = (0:L-1)' / fs;
            clat(i:i+L-1) = clat(i:i+L-1) + (0.5 + rand) * randn(L, 1) .* exp(-tt / 0.004);
        end
        tk = tk + (1 / ftr) * (1 + 0.1 * randn);
    end
    clat = biquad(clat, 'bp', 2500, 1.0, fs);
    n = n + 0.6 * unit_rms(clat);
    n = n + 0.3 * sin(2*pi*100*t);                        % alternator hum
end

function n = wind_noise(N, fs)
    b = biquad(brown_noise(N), 'lp', 500, 0.7, fs);
    g = filter(0.0005, [1 -0.9995], randn(N, 1));
    g = 1 + 2 * abs(g) / max(abs(g) + eps);
    n = unit_rms(b) .* g;
end

% =========================================================================
%  helpers
% =========================================================================
function n = pink_noise(N)
    b = [0.049922035 -0.095993537 0.050612699 -0.004408786];
    a = [1 -2.494956002 2.017265875 -0.522189400];
    n = filter(b, a, randn(N + 2000, 1));
    n = unit_rms(n(2001:end));
end

function n = brown_noise(N)
    n = filter(1, [1 -0.995], randn(N + 4000, 1));
    n = unit_rms(n(4001:end));
end

function y = biquad(x, type, fc, Q, fs)
% RBJ-cookbook biquad ('lp', 'hp', 'bp'), no toolbox needed.
    w0 = 2*pi*fc/fs;
    al = sin(w0) / (2*Q);
    c = cos(w0);
    switch type
        case 'lp', b = [(1-c)/2, 1-c, (1-c)/2];
        case 'hp', b = [(1+c)/2, -(1+c), (1+c)/2];
        case 'bp', b = [al, 0, -al];
    end
    a = [1+al, -2*c, 1-al];
    y = filter(b / a(1), a / a(1), x);
end

function tt = poisson_times(dur, rate, min_gap)
    tt = [];
    tc = 0.05 + (-log(rand) / rate);
    while tc < dur
        tt(end+1) = tc; %#ok<AGROW>
        tc = tc + max(min_gap, -log(rand) / rate);
    end
end

function n = add_event(n, e, t0, fs, g)
    i = round(t0 * fs) + 1;
    L = min(numel(e), numel(n) - i + 1);
    if L > 0
        n(i:i+L-1) = n(i:i+L-1) + g * e(1:L);
    end
end

function y = unit_rms(x)
    r = rms(x);
    if r > 0, y = x / r; else, y = x; end
end

function r = rms(x)
    r = sqrt(mean(x(:).^2));
end

function g = db2mag(d)
    g = 10.^(d / 20);
end

function s = rng_push(seed)
    s = rng();
    rng(seed);
end

function rng_pop(s)
    rng(s);
end
