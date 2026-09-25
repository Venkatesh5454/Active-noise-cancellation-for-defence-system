function [m, h] = dnn_mask_step(P, lam_s, h, M)
%DNN_MASK_STEP  One frame of the causal GRU mask estimator (AI/ML stage).
%
%   [m, h] = dnn_mask_step(P, lam_s, h, M)
%
%   P     : |Y(k)|^2 of the current frame (257 x 1)
%   lam_s : stationary-noise PSD from the SPP-MMSE tracker (257 x 1)
%   h     : GRU state (H x 1), zeros at start-up
%   M     : network weights (struct loaded from models/defence_gru.mat)
%   m     : spectral gain / mask in [0, 1] (257 x 1)
%
%   Network: features [log P ; log lam_s] -> dense(tanh) -> GRU -> dense
%   (sigmoid).  Pure matrix algebra, so it runs in any MATLAB / Octave
%   without the Deep Learning Toolbox, and maps 1:1 to the ONNX export
%   (models/defence_gru.onnx) for Jetson / TensorRT deployment.

    f  = [log(P + 1e-10); log(lam_s + 1e-10)];
    z  = tanh(M.W1 * ((f - M.mu) ./ M.sd) + M.b1);
    H  = numel(h);
    gi = M.Wih * z + M.bih;                 % gate order: reset, update, new
    gh = M.Whh * h + M.bhh;
    r  = 1 ./ (1 + exp(-(gi(1:H) + gh(1:H))));
    u  = 1 ./ (1 + exp(-(gi(H+1:2*H) + gh(H+1:2*H))));
    n  = tanh(gi(2*H+1:3*H) + r .* gh(2*H+1:3*H));
    h  = (1 - u) .* n + u .* h;
    m  = 1 ./ (1 + exp(-(M.W2 * h + M.b2)));
end
