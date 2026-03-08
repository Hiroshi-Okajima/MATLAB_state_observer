%% main_observer_feedback.m
%  Observer-Based State Feedback Control — Separation Principle
%
%  Demonstrates:
%    1. State feedback u = -K*x   (full state available — ideal case)
%    2. Observer-based feedback u = -K*xhat  (only output y is available)
%    3. Separation principle: K and L can be designed independently
%       - Closed-loop poles = {eig(A-BK)} ∪ {eig(A-LC)}
%
%  Plant (discrete-time, 2nd-order):
%    x(k+1) = A*x(k) + B*u(k)
%    y(k)   = C*x(k)
%
%  Controller:  u(k) = -K*xhat(k)
%  Observer:    xhat(k+1) = (A - L*C)*xhat(k) + B*u(k) + L*y(k)
%
%  Repository: MATLAB_state_observer/02_observer_based_control/
%  Blog: https://blog.control-theory.com/entry/2024/10/01/143305
%  Hub:  https://blog.control-theory.com/entry/state-observer-estimation
%
%  Author: Hiroshi Okajima, Kumamoto University
%  -----------------------------------------------------------------------

clear; clc; close all;

%% ---- Plant Parameters ---------------------------------------------------

% Same mass-spring-damper as 01_luenberger_observer
%   m = 1, c = 0.5, k = 2,  Ts = 0.1 s

Ts = 0.1;

Ac = [0, 1; -2, -0.5];
Bc = [0; 1];
Cc = [1, 0];

sys_c = ss(Ac, Bc, Cc, 0);
sys_d = c2d(sys_c, Ts, 'zoh');

A = sys_d.A;
B = sys_d.B;
C = sys_d.C;

n = size(A, 1);   % state dimension
m = size(B, 2);   % input dimension

fprintf('---- Plant (discrete, Ts = %.2f s) ----\n', Ts);
fprintf('A = \n'); disp(A);
fprintf('B = \n'); disp(B);
fprintf('C = \n'); disp(C);
fprintf('Open-loop poles: '); disp(eig(A).');

%% ---- Check Controllability and Observability ----------------------------

Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('Controllability rank = %d (n = %d)\n', rank(Co), n);
fprintf('Observability rank   = %d (n = %d)\n', rank(Ob), n);

if rank(Co) < n
    error('System is not controllable.');
end
if rank(Ob) < n
    error('System is not observable.');
end

%% ---- Controller Gain Design (Pole Placement) ----------------------------

% Desired closed-loop poles (controller)
ctrl_poles = [0.7, 0.65];
K = place(A, B, ctrl_poles);

fprintf('\n---- Controller ----\n');
fprintf('Desired closed-loop poles: '); disp(ctrl_poles);
fprintf('K = '); disp(K);
fprintf('Eigenvalues of A - B*K: '); disp(eig(A - B*K).');

%% ---- Observer Gain Design (Pole Placement) ------------------------------

% Observer poles: faster than controller poles
obs_poles = [0.2, 0.15];
L = place(A', C', obs_poles)';

fprintf('---- Observer ----\n');
fprintf('Desired observer poles: '); disp(obs_poles);
fprintf('L = \n'); disp(L);
fprintf('Eigenvalues of A - L*C: '); disp(eig(A - L*C).');

%% ---- Verification: Separation Principle ---------------------------------

% Augmented system: z = [x; e],  e = x - xhat
%   z(k+1) = [A-BK, BK; 0, A-LC] * z(k)
% Eigenvalues = {eig(A-BK)} ∪ {eig(A-LC)}

A_aug = [A - B*K, B*K; zeros(n), A - L*C];
eig_aug = eig(A_aug);

fprintf('\n---- Separation Principle Verification ----\n');
fprintf('Augmented system eigenvalues:\n');
disp(eig_aug);
fprintf('These should equal {eig(A-BK)} U {eig(A-LC)} = ');
disp(sort([ctrl_poles, obs_poles]).');

%% ---- Simulation ---------------------------------------------------------

T = 100;   % number of steps

% Initial conditions
x0    = [2; 0];     % true initial state (displaced position)
xhat0 = [0; 0];     % observer starts with zero estimate

% Reference: regulation to origin (r = 0)

% Storage
x_sf    = zeros(n, T);   % state feedback (ideal)
x_obf   = zeros(n, T);   % observer-based feedback
xhat    = zeros(n, T);   % observer estimate
x_open  = zeros(n, T);   % open-loop (no control)
u_sf    = zeros(m, T);
u_obf   = zeros(m, T);

x_sf(:,1)   = x0;
x_obf(:,1)  = x0;
xhat(:,1)   = xhat0;
x_open(:,1) = x0;

for k = 1:T-1
    % (a) Open-loop (no control)
    x_open(:,k+1) = A*x_open(:,k);

    % (b) State feedback (ideal: full state known)
    u_sf(k) = -K*x_sf(:,k);
    x_sf(:,k+1) = A*x_sf(:,k) + B*u_sf(k);

    % (c) Observer-based feedback
    y_k = C*x_obf(:,k);
    u_obf(k) = -K*xhat(:,k);
    x_obf(:,k+1) = A*x_obf(:,k) + B*u_obf(k);
    xhat(:,k+1)  = A*xhat(:,k) + B*u_obf(k) + L*(y_k - C*xhat(:,k));
end

% Estimation error
e_obs = x_obf - xhat;

% Time axis
t = (0:T-1) * Ts;

%% ---- Plots --------------------------------------------------------------

figure('Position', [100 100 900 850])

% (1) State x1 (position)
subplot(4, 1, 1)
stairs(t, x_open(1,:), 'k:', 'LineWidth', 1); hold on
stairs(t, x_sf(1,:), 'b', 'LineWidth', 2)
stairs(t, x_obf(1,:), 'r--', 'LineWidth', 1.5)
stairs(t, xhat(1,:), 'm-.', 'LineWidth', 1)
yline(0, 'k:')
legend('Open-loop', 'State FB (u=-Kx)', 'Observer FB (u=-K xhat)', ...
       'xhat_1', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_1 (position)')
title('Position x_1')
grid on

% (2) State x2 (velocity)
subplot(4, 1, 2)
stairs(t, x_open(2,:), 'k:', 'LineWidth', 1); hold on
stairs(t, x_sf(2,:), 'b', 'LineWidth', 2)
stairs(t, x_obf(2,:), 'r--', 'LineWidth', 1.5)
stairs(t, xhat(2,:), 'm-.', 'LineWidth', 1)
yline(0, 'k:')
legend('Open-loop', 'State FB', 'Observer FB', 'xhat_2', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_2 (velocity)')
title('Velocity x_2 (unmeasured)')
grid on

% (3) Control input
subplot(4, 1, 3)
stairs(t, u_sf, 'b', 'LineWidth', 1.5); hold on
stairs(t, u_obf, 'r--', 'LineWidth', 1.5)
yline(0, 'k:')
legend('u = -Kx (ideal)', 'u = -K xhat (observer-based)', 'Location', 'best')
xlabel('Time [s]')
ylabel('u')
title('Control Input')
grid on

% (4) Estimation error
subplot(4, 1, 4)
stairs(t, vecnorm(e_obs), 'b', 'LineWidth', 1.5)
yline(0, 'k:')
xlabel('Time [s]')
ylabel('||x - xhat||')
title('Observer Estimation Error Norm')
grid on

sgtitle('Observer-Based Feedback Control (Separation Principle)', ...
        'FontSize', 14, 'FontWeight', 'bold')

%% ---- Pole Map -----------------------------------------------------------

figure('Position', [1050 100 450 400])
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 0.5); hold on
plot(real(eig(A)), imag(eig(A)), 'kx', 'MarkerSize', 14, 'LineWidth', 2)
plot(real(ctrl_poles), imag(ctrl_poles), 'bs', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(obs_poles), imag(obs_poles), 'ro', 'MarkerSize', 10, 'LineWidth', 2)
legend('Unit circle', 'Open-loop poles', ...
       sprintf('Controller poles (%.2f, %.2f)', ctrl_poles), ...
       sprintf('Observer poles (%.2f, %.2f)', obs_poles), ...
       'Location', 'best')
xlabel('Real')
ylabel('Imaginary')
title('Closed-Loop Pole Locations (Separation Principle)')
axis equal; grid on
xlim([-1.3, 1.3]); ylim([-1.3, 1.3])
