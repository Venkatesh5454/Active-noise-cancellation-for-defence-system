function e = e1fast(v)
%E1FAST  Exponential integral E1(v) for v > 0 (Abramowitz & Stegun 5.1.53/56).
%   Relative error < 6e-5; fast and toolbox-free (used by the LSA gain).
    e = zeros(size(v));
    lo = v < 1;
    x = v(lo);
    e(lo) = -log(x) - 0.57721566 + x .* (0.99999193 + x .* (-0.24991055 + ...
            x .* (0.05519968 + x .* (-0.00976004 + x * 0.00107857))));
    x = v(~lo);
    e(~lo) = exp(-x) ./ x .* (x .* x + 2.334733 * x + 0.250621) ./ ...
             (x .* x + 3.330657 * x + 1.681534);
end
