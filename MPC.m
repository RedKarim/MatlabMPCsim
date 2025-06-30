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

%% MPC Parameters
Umn = -7;    % Minimum acceleration
Umx = 2;     % Maximum acceleration 
dt = 0.5;    % Time step
T = 20;      % Prediction horizon
cx = 3*(T+1)-1; % Total dimension (X, V, A)

%% Setup bounds
lb = nan(cx,1);
ub = lb;
lb(1:T+1) = -Inf;      % Position limits
ub(1:T+1) = Inf;
lb(T+2:2*T+2) = 0;     % Minimum speed
ub(T+2:2*T+2) = 18;    % Maximum speed
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
b(1:T,1) = XPT - 7;    % Gap with preceding vehicle
b(1:3,1) = b(1:3,1) + 4; % Additional safety margin for first few steps

A = zeros(T,cx); 
A(1:T,2:T+1) = eye(T);      % Position constraints
A(1:T,T+3:2*T+2) = 0.5*eye(T); % Acceleration influence

%% Solve optimization
options = optimset('Algorithm','sqp','GradObj','off','Display','off');
[x_opt,~,exitflag,~] = fmincon(@(x)ObjFn(x,XPT,T),xi,A,b,Aeq,Beq,lb,ub,[],options);

%% Extract optimal acceleration for current timestep
if exitflag > 0
    acc = x_opt(2*T+3);  % First acceleration command
else
    acc = 0;  % Fallback if optimization fails
    warning('MPC optimization failed, using zero acceleration');
end

% Bound the acceleration output
if acc < Umn
    acc = Umn;
elseif acc > Umx
    acc = Umx;
end

end

function cost = ObjFn(x, XPT, T)
% Objective function for MPC optimization
% Minimize tracking error and control effort

% Extract variables
pos = x(2:T+1);           % Positions (skip initial position)
vel = x(T+3:2*T+2);       % Velocities (skip initial velocity)
acc = x(2*T+3:end);       % Accelerations

% Weights
w_pos = 1;      % Position tracking weight
w_vel = 10;     % Velocity tracking weight  
w_acc = 0.1;    % Acceleration smoothness weight

% Target velocity (cruise speed)
v_target = 11;

% Cost components
pos_cost = w_pos * sum((pos - (XPT - 7)).^2);  % Track preceding vehicle with gap
vel_cost = w_vel * sum((vel - v_target).^2);   % Track target velocity
acc_cost = w_acc * sum(acc.^2);                % Minimize acceleration effort

cost = pos_cost + vel_cost + acc_cost;

end
