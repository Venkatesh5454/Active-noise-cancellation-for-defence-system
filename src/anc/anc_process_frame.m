function [out, st] = anc_process_frame(xin, st)
%ANC_PROCESS_FRAME  Process one hop of audio (real-time / streaming core).
%
%   [out, st] = anc_process_frame(xin, st)
%
%   xin : st.R new input samples (column)      out : st.R output samples
%   The output is delayed by st.N - st.R samples (24 ms) w.r.t. the input;
%   total algorithmic latency is one frame (32 ms).
%
%   Per frame:
%     1. STFT analysis (sqrt-Hann, 75 % overlap)
%     2. stationary noise PSD      - SPP-based MMSE tracker
%     3. impulsive noise PSD       - broadband-onset detector + excess over
%                                    a peak-hold speech/noise reference
%     4. tonal noise PSD           - persistent narrow-band peak tracker
%     5. OM-LSA gain               - MMSE log-spectral amplitude estimator,
%                                    decision-directed a-priori SNR, speech
%                                    presence weighting, noise-relative floor
%     6. ISTFT overlap-add

    p = st.p;
    K = st.K;
    st.frame = st.frame + 1;

    % ---- 1. analysis ------------------------------------------------------
    st.inbuf = [st.inbuf(st.R+1:end); xin(:)];
    Yf = fft(st.inbuf .* st.win);
    Y  = Yf(1:K);
    P  = max(real(Y).^2 + imag(Y).^2, 1e-12);

    % ---- 2. stationary noise tracker ------------------------------------
    if st.frame <= st.n_init
        st.lambda_s = st.lambda_s + (P - st.lambda_s) / st.frame;
        if st.frame == 1
            st.Rref = P;  st.Rhold = P;
        end
    else
        snr_post = P ./ st.lambda_s;
        glr = st.prior_fact * exp(min(st.log_glr_fact + st.glr_exp * snr_post, 200));
        ph1 = glr ./ (1 + glr);
        st.PH1mean = st.a_p * st.PH1mean + (1 - st.a_p) * ph1;
        stuck = st.PH1mean > 0.99;
        ph1(stuck) = min(ph1(stuck), 0.99);
        est = ph1 .* st.lambda_s + (1 - ph1) .* P;
        st.lambda_s = st.a_psd * st.lambda_s + (1 - st.a_psd) * est;
    end
    lam_s = max(st.lambda_s, 1e-12);

    % ---- AI/ML stage: GRU mask estimator (replaces steps 3-5) -------------
    if ~isempty(st.dnn)
        [m, st.h] = dnn_mask_step(P, lam_s, st.h, st.dnn);
        st.last_mask = m;
        G = max(m, st.dnn_floor) .* st.lowcut;
        S  = G .* Y;
        fr = real(ifft([S; conj(S(K-1:-1:2))])) .* st.win;
        st.outbuf = st.outbuf + fr;
        out = st.outbuf(1:st.R) * st.ola_norm;
        st.outbuf = [st.outbuf(st.R+1:end); zeros(st.R, 1)];
        return;
    end

    % ---- 3. impulsive (transient) noise ---------------------------------
    lam_t = zeros(K, 1);
    Yuse = Y;
    Puse = P;
    pt = 0;
    if p.transient && st.frame > 1
        b = st.tr_bins;
        rise = sort(10 * log10(P(b) ./ max(st.Rhold(b), 1e-12)));
        Q = rise(max(1, round(p.tr_quantile * numel(b))));
        excess = 10 * log10(sum(P(b)) / sum(lam_s(b)));
        if excess >= p.tr_min_excess_db
            pt = min(max((Q - p.tr_off_db) / (p.tr_on_db - p.tr_off_db), 0), 1);
        end
        if pt > 0
            if strcmp(p.tr_mode, 'restore')
                lim = sqrt(min(1, p.tr_over * st.Rref ./ P));
                fac = (1 - pt) + pt * lim;
                Yuse = Y .* fac;
                Puse = P .* fac.^2;
            else
                lam_t = pt * max(P - p.tr_over * st.Rref, 0);
            end
        end
    end
    if pt < 0.5 || st.tr_count > st.tr_max
        st.Rref = st.a_ref * st.Rref + (1 - st.a_ref) * P;
    end
    st.Rhold = max(st.Rref, st.Rhold * st.hold_dec);
    if pt >= 0.5
        st.tr_count = st.tr_count + 1;
        st.stat_tr_frames = st.stat_tr_frames + 1;
    else
        st.tr_count = 0;
    end
    st.pt = pt;

    % ---- 4. tonal noise (persistent narrow-band peaks) --------------------
    lam_tone = zeros(K, 1);
    if p.tonal
        med = median(Puse(st.med_idx), 2);
        pk = (Puse > med * 10^(p.ton_peak_db / 10)) & st.ton_ok & ...
             (Puse >= [Puse(2:end); 0]) & (Puse >= [0; Puse(1:end-1)]);
        Td = max([st.T, [0; st.T(1:end-1)], [st.T(2:end); 0]], [], 2);
        st.T = pk .* (st.a_ton * Td + (1 - st.a_ton)) + (~pk) .* (0.85 * st.T);
        tone = pk & (st.T > p.ton_thresh);
        if any(tone)
            w = p.ton_width;
            msk = conv(double(tone), ones(2*w+1, 1), 'same') > 0;
            lam_tone(msk) = Puse(msk);
            st.stat_ton_frames = st.stat_ton_frames + 1;
        end
    end

    % ---- 5. OM-LSA gain -----------------------------------------------------
    lam   = lam_s + lam_t + lam_tone;
    if isfield(st, 'oracle_P')                 % research: true noise PSD
        lam = max(st.oracle_P, 1e-12);
        lam_s = lam;
    end
    gamma = min(Puse ./ lam, 1e4);
    xi    = st.a_dd * st.A2prev ./ lam + (1 - st.a_dd) * max(gamma - 1, 0);
    xi    = max(xi, st.xi_min);
    v     = max(xi ./ (1 + xi) .* gamma, 1e-8);
    Glsa  = xi ./ (1 + xi) .* exp(0.5 * e1fast(v));
    ps    = 1 ./ (1 + st.qfac * (1 + xi) .* exp(-min(v, 500)));
    Gf    = st.gmin * sqrt(min(1, lam_s ./ Puse));
    G     = max(Glsa .^ ps .* Gf .^ (1 - ps), Gf);
    G     = min(G, 1);
    if p.gain_smooth
        Gs = conv(G, [0.25; 0.5; 0.25], 'same');
        Gs([1 K]) = G([1 K]);
        G = ps .* G + (1 - ps) .* Gs;      % smooth only where speech is unlikely
    end
    G = G .* st.lowcut;
    st.A2prev = G.^2 .* Puse;

    % ---- 6. synthesis -----------------------------------------------------
    S  = G .* Yuse;
    fr = real(ifft([S; conj(S(K-1:-1:2))])) .* st.win;
    st.outbuf = st.outbuf + fr;
    out = st.outbuf(1:st.R) * st.ola_norm;
    st.outbuf = [st.outbuf(st.R+1:end); zeros(st.R, 1)];
end
