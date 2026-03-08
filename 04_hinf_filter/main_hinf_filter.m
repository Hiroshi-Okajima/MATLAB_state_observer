%% main_hinf_filter.m
%  H-infinity Filter Design via LMI
%
%  Demonstrates:
%    1. H-infinity filter design using Linear Matrix Inequalities (LMIs)
%    2. Minimizes worst-case l2-induced norm from disturbance to
%       estimation error (no statistical noise assumptions required)
%    3. State estimation under unknown-but-bounded disturbances
%
%  Plant:
%    x(k+1) = A*x(k) + Bd*w(k)
%    y(k)   = C*x(k) + Dd*w(k)
%    z(k)   = Lz*x(k)                  (estimation target)
%
%  Filter (Luenberger-type):
%    xhat(k+1) = A*xhat(k) + L*(y(k) - C*xhat(k))
%              = (A + L*C)*xhat(k) + B*u(k) - L*y(k)
%              (sign convention: L includes negative sign, same as
%               multirate observer code where L = -P\Y*S)
%
%  Error system: e = x - xhat
%    e(k+1) = (A + L*C)*e(k) + (Bd + L*Dd)*w(k)
%    ez(k)  = Lz*e(k)
%
%  LMI (Bounded Real Lemma):
%    Variables: gamma2 (scalar), P > 0 (n x n), Y (n x p)
%    where Y = P*L
%
%    [ P,          (P*A+Y*C)',   Lz',      0           ]
%    [ P*A+Y*C,    P - I,        0,        P*Bd+Y*Dd   ] > 0
%    [ Lz,         0,            I,        0           ]
%    [ 0,          (P*Bd+Y*Dd)', 0,        gamma2*I    ]
%
%    minimize gamma2
%    Recover L = P \ Y
%
%  Requires: Robust Control Toolbox (for setlmis, lmivar, lmiterm, mincx)
%
%  Repository: MATLAB_state_observer/04_hinf_filter/
%  Blog: https://blog.control-theory.com/entry/h-infinity-filter
%  Blog: https://blog.control-theory.com/entry/lmi-eng
%  Hub:  https://blog.control-theory.com/entry/state-observer-estimation
%  Research: https://www.control-theory.com/en/linear-matrix-inequality
%
%  Related code:
%    LMI basics: https://github.com/Hiroshi-Okajima/MATLAB_fandamental_control_LMI
%    Multirate observer (LMI): File Exchange 182941
%      https://jp.mathworks.com/matlabcentral/fileexchange/182941
%
%  Author: Hiroshi Okajima, Kumamoto University
%  -----------------------------------------------------------------------

clear; clc; close all;

%% ---- Plant Parameters ---------------------------------------------------

% 2nd-order system (mass-spring-damper, discretized)
%   m = 1, c = 0.5, k = 2,  Ts = 0.1 s
% Same plant as 01_luenberger_observer and 03_kalman_filter

Ts = 0.1;

Ac = [0, 1; -2, -0.5];
Bc_ctrl = [0; 1];
Cc = [1, 0];

sys_c = ss(Ac, Bc_ctrl, Cc, 0);
sys_d = c2d(sys_c, Ts, 'zoh');

A = sys_d.A;
C = sys_d.C;

n = size(A, 1);       % state dimension = 2
p = size(C, 1);       % output dimension = 1

% Disturbance structure
%   w = [w1; w2] where w1: process noise, w2: measurement noise
%   x(k+1) = A*x(k) + Bd*w(k)
%   y(k)   = C*x(k) + Dd*w(k)

Bd = [0.1, 0;
      0,   0.1];     % process disturbance on both states
Dd = [0, 0.3];       % measurement disturbance

nw = size(Bd, 2);    % disturbance dimension = 2

% Estimation target: full state
Lz = eye(n);
nz = size(Lz, 1);   % = n = 2

fprintf('==== Plant (discrete-time, Ts = %.2f s) ====\n', Ts);
fprintf('n = %d,  p = %d,  nw = %d\n', n, p, nw);
fprintf('A =\n'); disp(A);
fprintf('C  = '); disp(C);
fprintf('Bd =\n'); disp(Bd);
fprintf('Dd = '); disp(Dd);

%% ---- H-infinity Filter Design via LMI ----------------------------------
%
%  Following the same LMI coding style as the multirate observer code
%  (File Exchange 182941, IEEE Access 2023).
%
%  LMI:
%    Theta = [ P,          (P*A+Y*C)',   Lz',      0           ]
%            [ P*A+Y*C,    P - I,        0,        P*Bd+Y*Dd   ] > 0
%            [ Lz,         0,            I,        0           ]
%            [ 0,          (P*Bd+Y*Dd)', 0,        gamma2*I    ]
%
%  Decision variables:
%    gamma2 (scalar, declared FIRST for c = [1, 0, ...] objective)
%    P      (n x n, symmetric)
%    Y      (n x p, full)

fprintf('\n==== Solving LMI ====\n');

setlmis([]);

% gamma2 declared FIRST so that c = [1, 0, 0, ...] minimizes gamma2
[gamma2, ~, sgamma2] = lmivar(1, [1 1]);     % scalar gamma^2
[P,      ~, sP]      = lmivar(1, [n 1]);     % symmetric P (n x n)
[Y,      ~, sY]      = lmivar(2, [n p]);     % full Y (n x p)

% LMI: Theta > 0  (using -lmi for > 0 convention)
%   Block 1: n x n    (rows/cols 1..n)
%   Block 2: n x n    (rows/cols n+1..2n)
%   Block 3: nz x nz  (rows/cols 2n+1..2n+nz)
%   Block 4: nw x nw  (rows/cols 2n+nz+1..2n+nz+nw)

lmi1 = newlmi;

% (1,1): P
lmiterm([-lmi1, 1, 1, P], 1, 1);

% (2,1): P*A + Y*C   (lower-left of block, MATLAB handles transpose for upper)
lmiterm([-lmi1, 2, 1, P], 1, A);       % P*A
lmiterm([-lmi1, 2, 1, Y], 1, C);       % Y*C

% (2,2): P - I
lmiterm([-lmi1, 2, 2, P], 1, 1);       % P
lmiterm([-lmi1, 2, 2, 0], -eye(n));    % -I

% (3,1): Lz
lmiterm([-lmi1, 3, 1, 0], Lz);

% (3,3): I
lmiterm([-lmi1, 3, 3, 0], eye(nz));

% (2,4): P*Bd + Y*Dd
lmiterm([-lmi1, 2, 4, P], 1, Bd);      % P*Bd
lmiterm([-lmi1, 2, 4, Y], 1, Dd);      % Y*Dd

% (4,4): gamma2 * I
lmiterm([-lmi1, 4, 4, gamma2], 1, eye(nw));

LMIs = getlmis;

% Objective: minimize gamma2 (first decision variable)
ndec = decnbr(LMIs);
c = zeros(1, ndec);
c(1) = 1;    % gamma2 is the first variable

fprintf('Number of decision variables: %d\n', ndec);

[copt, xopt] = mincx(LMIs, c);

if isempty(xopt)
    error('LMI is infeasible. Check plant parameters.');
end

% Extract results
P_val      = dec2mat(LMIs, xopt, P);
Y_val      = dec2mat(LMIs, xopt, Y);
gamma2_val = dec2mat(LMIs, xopt, gamma2);

% Recover filter gain: L = P \ Y (sign convention: L includes negative)
%   xhat(k+1) = (A + L*C)*xhat(k) + B*u - L*y
%   i.e. xhat(k+1) = A*xhat - L*(y - C*xhat)   (L is negative of usual F)
L = P_val \ Y_val;

gamma = sqrt(gamma2_val);

fprintf('\ngamma (optimal) = %.6f\n', gamma);
fprintf('L =\n'); disp(L);
fprintf('eig(A + L*C) = '); disp(eig(A + L*C).');

% Verify: eigenvalues inside unit circle
eig_err = eig(A + L*C);
if all(abs(eig_err) < 1)
    fprintf('  -> All eigenvalues inside unit circle: STABLE\n');
else
    warning('Error system is NOT stable!');
end

%% ---- Simulation ---------------------------------------------------------

T = 200;     % number of steps
rng(42);     % reproducibility

% Initial conditions
x0    = [2; -1];    % true initial state
xhat0 = [0;  0];    % filter initial estimate

% Input signal (for demonstration)
u_signal = 0.5 * sin(0.2 * (0:T-1));

% Storage
x    = zeros(n, T);
y    = zeros(p, T);
xhat = zeros(n, T);

x(:,1)    = x0;
xhat(:,1) = xhat0;

for k = 1:T-1
    % Disturbance (unknown-but-bounded)
    w_k = randn(nw, 1);

    % True system
    x(:,k+1) = A*x(:,k) + sys_d.B*u_signal(k) + Bd*w_k;
    y(k)     = C*x(:,k) + Dd*w_k;

    % H-infinity filter
    %   xhat(k+1) = A*xhat(k) + B*u(k) - L*(y(k) - C*xhat(k))
    %             = (A + L*C)*xhat(k) + B*u(k) - L*y(k)
    xhat(:,k+1) = A*xhat(:,k) + sys_d.B*u_signal(k) - L*(y(k) - C*xhat(:,k));
end
y(T) = C*x(:,T) + Dd*randn(nw,1);

% Estimation error
e = x - xhat;

% Time axis
t = (0:T-1) * Ts;

% Performance evaluation
rmse = sqrt(mean(sum(e.^2, 1)));
fprintf('\nRMSE = %.4f\n', rmse);

%% ---- Plots --------------------------------------------------------------

figure('Position', [100 100 850 750])

% (1) State x1 (position): true and estimate
subplot(3, 1, 1)
stairs(t, x(1,:), 'k', 'LineWidth', 2); hold on
stairs(t, xhat(1,:), 'b--', 'LineWidth', 1.5)
plot(t, y, 'r.', 'MarkerSize', 5)
legend('True x_1', 'H-inf estimate', 'Measurement y', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_1 (position)')
title('State x_1 Estimation')
grid on

% (2) State x2 (velocity, unmeasured)
subplot(3, 1, 2)
stairs(t, x(2,:), 'k', 'LineWidth', 2); hold on
stairs(t, xhat(2,:), 'b--', 'LineWidth', 1.5)
legend('True x_2', 'H-inf estimate', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_2 (velocity)')
title('State x_2 Estimation (unmeasured state)')
grid on

% (3) Estimation error
subplot(3, 1, 3)
stairs(t, e(1,:), 'b', 'LineWidth', 1); hold on
stairs(t, e(2,:), 'r', 'LineWidth', 1)
stairs(t, vecnorm(e), 'k', 'LineWidth', 1.5)
yline(0, 'k:')
legend('e_1', 'e_2', '||e||', 'Location', 'best')
xlabel('Time [s]')
ylabel('Estimation error')
title(sprintf('Estimation Error (gamma = %.4f)', gamma))
grid on

sgtitle(sprintf('H-infinity Filter via LMI (gamma = %.4f)', gamma), ...
        'FontSize', 14, 'FontWeight', 'bold')
