function [acc, x_opt] = MPC(Xh, Vh, Xp, Vp)
% MPC - Model Predictive Control for vehicle following
% Inputs:
%   Xh - Host vehicle position
%   Vh - Host vehicle velocity  
%   Xp - Preceding vehicle position
%   Vp - Preceding vehicle velocity
% Outputs:
%   acc - Optimal acceleration for current timestep
%   x_opt - Full optimal trajectory (optional)

%% MPC Parameters - match Python behavior
Umn = -5;    % Minimum acceleration
Umx = 2;     % Maximum acceleration 
dt = 0.5;    % Time step
T = 8;       % Prediction horizon (match Python CasADiMPC)
cx = 3*(T+1)-1; % Total dimension (X, V, A)

%% Setup bounds
lb = nan(cx,1);
ub = lb;
lb(1:T+1) = -Inf;      % Position limits
ub(1:T+1) = Inf;
lb(T+2:2*T+2) = 0;     % Minimum speed
ub(T+2:2*T+2) = 18;    % Maximum speed (match Python)
lb(2*T+3:end) = Umn;   % Minimum acceleration
ub(2*T+3:end) = Umx;   % Maximum acceleration

%% Setup equality constraints (dynamics)
Ax = 1.0; 
Bx = dt; 
dt2 = 0.5*dt^2;
Aeq = zeros(cx,cx);

T1 = -eye(T+1); 
T1(2:end,1:T) = eye(T)*Ax + T1(2:end,1:T);
Aeq(1:T+1,1:T+1) = T1;
Aeq(T+2:2*T+2,T+2:2*T+2) = T1;
Aeq(2:T+1,T+2:2*T+1) = eye(T)*Bx; 
Aeq(2:T+1,2*T+3:3*T+2) = eye(T)*dt2;
Aeq(T+3:2*T+2,2*T+3:3*T+2) = eye(T)*dt; 

%% Setup equality constraint vector
Beq = zeros(cx,1); 
Beq(1) = -Xh;   % Initial position
Beq(T+2) = -Vh; % Initial velocity

%% Determine target velocity based on scenario (match Python logic)
if Vp == 0 && (Xp - Xh) < 50 && (Xp - Xh) > 0
    % Traffic signal/pedestrian scenario - be more conservative
    v_target = min(8, Vh + 2);  % Conservative target speed
    safety_gap = 5;
elseif Xp == Xh + 500
    % Free driving scenario
    v_target = 9;  % Match Python free driving target
    safety_gap = 3;
else
    % Normal car following
    v_target = min(Vp, 11);  % Match Python car following
    safety_gap = 3;
end

%% Predicted trajectory of preceding vehicle
XPT = Xp + ((1:T)*Vp*dt)';

%% Initial guess
xi = zeros(cx,1);
xi(1) = Xh;        % Initial position
xi(T+2) = Vh;      % Initial velocity
a0 = -0.5;         % Initial acceleration guess

for J = 1:T
    xi(J+1) = xi(J) + xi(T+1+J)*dt + 0.5*a0*dt^2;
    xi(T+1+J+1) = xi(T+1+J) + a0*dt;
end
xi(2*(T+1)+1:end) = a0;

%% Setup inequality constraints (safety gap)
b = zeros(T,1);
b(1:T,1) = XPT - safety_gap;    % Gap with preceding vehicle/signal

A = zeros(T,cx); 
A(1:T,2:T+1) = eye(T);      % Position constraints

%% Solve optimization with multiple attempts
options = optimset('Algorithm','interior-point','Display','off','MaxIter',100,'TolFun',1e-3);

% Try optimization with current initial guess
[x_opt,~,exitflag,~] = fmincon(@(x)ObjFn(x,XPT,T,Xh,Vh,Xp,Vp,v_target),xi,A,b,Aeq,Beq,lb,ub,[],options);

% If failed, try with simpler initial guess
if exitflag <= 0
    xi_simple = zeros(cx,1);
    xi_simple(1) = Xh;
    xi_simple(T+2) = Vh;
    % Simple constant velocity prediction
    for j = 1:T
        xi_simple(j+1) = xi_simple(j) + Vh*dt;
        xi_simple(T+1+j+1) = Vh;
    end
    [x_opt,~,exitflag,~] = fmincon(@(x)ObjFn(x,XPT,T,Xh,Vh,Xp,Vp,v_target),xi_simple,A,b,Aeq,Beq,lb,ub,[],options);
end

%% Extract optimal acceleration for current timestep
if exitflag > 0
    acc = x_opt(2*T+3);  % First acceleration command
else
    % Fallback using simple proportional controller (match Python fallback)
    gap_distance = Xp - Xh;
    if gap_distance > 0
        % Simple proportional controller for fallback
        desired_gap = 10;
        gap_error = gap_distance - desired_gap;
        acc = 0.5 * gap_error / 10;  % Proportional control
        acc = max(min(acc, Umx), Umn);  % Bound acceleration
    else
        acc = 0;
    end
end

% Bound the acceleration output
if acc < Umn
    acc = Umn;
elseif acc > Umx
    acc = Umx;
end

end

function cost = ObjFn(x, XPT, T, Xh, Vh, Xp, Vp, v_target)
% Improved objective function matching Python behavior
% Focus on smooth operation with minimal time at signals

% Extract variables
pos = x(2:T+1);           % Positions (skip initial position)
vel = x(T+3:2*T+2);       % Velocities (skip initial velocity)
acc = x(2*T+3:end);       % Accelerations

% Adaptive weights based on scenario (match Python weights)
gap_distance = Xp - Xh;
if Vp == 0 && gap_distance < 50 && gap_distance > 0
    % Traffic signal scenario - prioritize smooth approach
    w_vel = 5;      % Moderate velocity tracking
    w_acc = 2;      % Emphasize smooth acceleration
    w_jerk = 1;     % Minimize jerk for comfort
else
    % Normal following scenario
    w_vel = 10;     % Higher velocity tracking
    w_acc = 0.5;    % Less emphasis on acceleration
    w_jerk = 0.1;   % Minimal jerk weight
end

% Cost components (match Python cost structure)
vel_cost = w_vel * sum((vel - v_target).^2);   % Track target velocity
acc_cost = w_acc * sum(acc.^2);                % Minimize acceleration effort

% Jerk minimization (acceleration smoothness)
jerk_cost = 0;
if length(acc) > 1
    jerk = diff(acc);
    jerk_cost = w_jerk * sum(jerk.^2);
end

% Penalty for stopping (encourage smooth flow)
stop_penalty = sum((vel < 1).^2) * 5;

cost = vel_cost + acc_cost + jerk_cost + stop_penalty;

end
