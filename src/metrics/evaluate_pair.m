function r = evaluate_pair(clean, test, fs)
%EVALUATE_PAIR  [SNR (dB), STOI, PESQ-WB (P.862.2), PESQ-NB (P.862.1)] of test vs clean.
    m = snr_metrics(clean, test, fs);
    r = [m.snr, stoi_score(clean, test, fs), pesq_score(clean, test, fs, 'wb'), ...
         pesq_score(clean, test, fs, 'nb')];
end
