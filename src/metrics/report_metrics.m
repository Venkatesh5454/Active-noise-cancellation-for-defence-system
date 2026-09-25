function report_metrics(title_str, clean, noisy, enh, fs)
%REPORT_METRICS  Print SNR / STOI / PESQ of the noisy and the enhanced signal.
    a = evaluate_pair(clean, noisy, fs);
    b = evaluate_pair(clean, enh, fs);
    fprintf(' %s:\n', title_str);
    fprintf('   SNR     %7.2f dB -> %7.2f dB\n', a(1), b(1));
    fprintf('   STOI    %7.3f    -> %7.3f\n', a(2), b(2));
    fprintf('   PESQ-WB %7.2f    -> %7.2f\n', a(3), b(3));
    fprintf('   PESQ-NB %7.2f    -> %7.2f\n', a(4), b(4));
end
