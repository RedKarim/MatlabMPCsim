function [acc] = IDM(Xh, Vh, Xp, Vp)
% IDM - Intelligent Driver Model matching Python implementation
% Inputs:
%   Xh - Host vehicle position
%   Vh - Host vehicle velocity  
%   Xp - Preceding vehicle position
%   Vp - Preceding vehicle velocity
% Outputs:
%   acc - Optimal acceleration for current timestep

% IDM parameters - match Python exactly
v0 = 9;        % Desired speed (Python: v0 = 9)
T = 2;       % Time headway (Python: T = 1.5)
a_max = 2;     % Max acceleration (Python: a_max = 2)
b = 3;         % Comfortable deceleration (Python: b = 3)
s0 = 8;        % Minimum gap (Python: s0 = 2)
delta = 4;     % Acceleration exponent (Python: delta = 4)

% Check if this is free driving case (Python: x_lead == x_ego + 500)
if Xp == Xh + 500
    acc = a_max * (1 - (Vh / v0)^delta);
else
    % Following case
    s = Xp - Xh;
    s_star = s0 + Vh * T + (Vh * (Vh - Vp)) / (2 * sqrt(a_max * b));
    acc = a_max * (1 - (Vh / v0)^delta - (s_star / s)^2);
end

% Bound acceleration (Python: max(min(car['a'], 2.5), -5))
if acc < -5
    acc = -5;
elseif acc > 2.5
    acc = 2.5;
end

end

