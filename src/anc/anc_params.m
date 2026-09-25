function p = anc_params(varargin)
%ANC_PARAMS  Default parameters of the hybrid adaptive noise canceller.
%
%   p = anc_params()                     defaults
%   p = anc_params('name', value, ...)   override individual fields
%
%   All time constants are specified for the default 16 kHz / 8 ms hop and
%   are converted to per-frame values inside anc_init.m.

    % ---- framework -------------------------------------------------------
    p.fs        = 16000;   % processing rate (Hz)
    p.frame_ms  = 32;      % STFT frame (algorithmic latency)
    p.hop_ms    = 8;       % hop size (75 % overlap)

    % ---- stage 1: pre-filter --------------------------------------------
    p.dc_block  = true;    % 1st-order DC blocker (~10 Hz, negligible phase)
    p.lowcut_hz = 70;      % zero-phase spectral low-cut for rumble / wind
    p.lowcut_db = -30;     % attenuation below lowcut_hz

    % ---- AI/ML stage: causal GRU mask estimator --------------------------
    p.use_dnn      = true;   % use models/defence_gru.mat if it exists
    p.model_file   = '';     % '' = default location
    p.dnn_floor_db = -30;    % lowest gain applied by the network mask

    % ---- stage 2: stationary noise tracker (SPP-MMSE) ---------------------
    p.spp_xi_h1_db   = 15;     % fixed a-priori SNR under speech presence
    p.spp_alpha_psd  = 0.80;   % PSD smoothing  (per 16 ms)
    p.spp_alpha_p    = 0.90;   % SPP smoothing  (per 16 ms)
    p.init_ms        = 64;     % initial noise estimate from first frames

    % ---- stage 3: impulsive noise (gunshot / MG / explosion) ------------
    p.transient       = true;
    p.tr_band_hz      = [300 7000];  % detection band
    p.tr_quantile     = 0.30;   % fraction of bins that may NOT rise
    p.tr_on_db        = 5.0;    % broadband rise => transient
    p.tr_off_db       = 1.5;    % rise below this => no transient
    p.tr_min_excess_db= 6.0;    % frame must exceed stationary noise by this
    p.tr_ref_ms       = 24;     % reference (speech + noise) smoothing
    p.tr_hold_db_s    = 40;     % decay of the peak-hold reference (dB/s)
    p.tr_max_ms       = 400;    % max. duration before reference re-adapts
    p.tr_over         = 1.0;    % over-subtraction of the transient PSD
    p.tr_mode         = 'suppress';  % 'suppress' or 'restore'

    % ---- stage 4: tonal noise (whine, sirens, rotor / engine hum) -------
    p.tonal           = true;
    p.ton_peak_db     = 9;      % peak-to-local-median ratio
    p.ton_persist_ms  = 400;    % time to confirm a persistent tone
    p.ton_thresh      = 0.6;    % persistence threshold (0..1)
    p.ton_width       = 2;      % notch half-width (bins)

    % ---- stage 5: OM-LSA gain -------------------------------------------
    p.dd_alpha       = 0.96;    % decision-directed smoothing (per 8 ms)
    p.xi_min_db      = -25;     % a-priori SNR floor
    p.q_absence      = 0.5;     % a-priori speech absence probability
    p.gmin_db        = -20;     % residual floor re. stationary noise level
    p.gain_smooth    = true;    % cross-frequency smoothing of the gain

    % ---- stage 6: optional reference-microphone NLMS ANC ------------------
    p.nlms_taps      = 256;     % 16 ms adaptive FIR
    p.nlms_mu        = 0.3;
    p.nlms_vad_gate  = true;    % freeze adaptation during speech

    % ---- output ---------------------------------------------------------
    p.out_peak       = 0.95;    % peak normalisation limit (full scale = 1)

    for i = 1:2:numel(varargin)
        p.(varargin{i}) = varargin{i+1};
    end
end
