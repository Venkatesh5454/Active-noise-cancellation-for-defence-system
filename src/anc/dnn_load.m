function M = dnn_load(file)
%DNN_LOAD  Load the trained GRU mask estimator (models/defence_gru.mat).
%   M = dnn_load()        searches the usual places for defence_gru.mat
%   M = dnn_load(file)    loads a specific model file
%   Returns [] if no model is found (the canceller then falls back to the
%   purely statistical OM-LSA filter).
    M = [];
    if nargin < 1 || isempty(file)
        here = fileparts(mfilename('fullpath'));
        cand = {fullfile(here, '..', '..', 'models', 'defence_gru.mat'), ...
                fullfile(here, 'models', 'defence_gru.mat'), ...
                fullfile(pwd, 'models', 'defence_gru.mat'), ...
                which('defence_gru.mat')};
        file = '';
        for i = 1:numel(cand)
            if ~isempty(cand{i}) && exist(cand{i}, 'file')
                file = cand{i};
                break;
            end
        end
    end
    if isempty(file) || ~exist(file, 'file'), return; end
    S = load(file);
    col = @(v) double(v(:));
    M.mu  = col(S.mu);    M.sd  = col(S.sd);
    M.W1  = double(S.W1); M.b1  = col(S.b1);
    M.Wih = double(S.Wih); M.Whh = double(S.Whh);
    M.bih = col(S.bih);   M.bhh = col(S.bhh);
    M.W2  = double(S.W2); M.b2  = col(S.b2);
    M.H   = size(M.Whh, 2);
end
