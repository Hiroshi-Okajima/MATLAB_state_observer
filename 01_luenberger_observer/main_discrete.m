%% main_discrete.m
%  Discrete-Time Luenberger Observer Demonstration
%
%  Demonstrates:
%    - State observer for a 2nd-order discrete-time LTI system
%    - Observer gain design via pole placement
%    - Convergence of estimation error e(k) = x(k) - xhat(k) -> 0
%    - Effect of observer pole locations on convergence speed
%
%  System model:
%    x(k+1) = A*x(k) + B*u(k)
%    y(k)   = C*x(k)
%
%  Observer:
%    xhat(k+1) = A*xhat(k) + B*u(k) + L*(y(k) - C*xhat(k))
%              = (A - L*C)*xhat(k) + B*u(k) + L*y(k)
%
%  Error dynamics:
%    e(k+1) = (A - L*C)*e(k)
%    -> e(k) -> 0 if A - L*C is Schur (all eigenvalues inside unit circle)
%
%  Repository: MATLAB_state_observer/01_luenberger_observer/
%  Blog: https://blog.control-theory.com/entry/2024/02/28/100201
%  Blog: https://blog.control-theory.com/entry/2024/10/01/143305
%  Hub:  https://blog.control-theory.com/entry/state-observer-estimation
%
%  Author: Hiroshi Okajima, Kumamoto University
%  -----------------------------------------------------------------------

clear; clc; close all;

%% ---- Plant Parameters ---------------------------------------------------

% 2nd-order discrete-time system (obtained by discretizing a
% mass-spring-damper with m=1, c=0.5, k=2, sampling period Ts=0.1)

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

fprintf('---- Plant (discrete, Ts = %.2f s) ----\n', Ts);
fprintf('A = \n'); disp(A);
fprintf('B = \n'); disp(B);
fprintf('C = \n'); disp(C);
fprintf('Plant poles: '); disp(eig(A).');

%% ---- Check Observability ------------------------------------------------

Ob = obsv(A, C);
fprintf('Observability matrix rank = %d (n = %d)\n', rank(Ob), n);

if rank(Ob) < n
    error('System is not observable. Observer design is not possible.');
end

%% ---- Observer Gain Design (Pole Placement) ------------------------------

% Compare two observer designs: slow and fast convergence
%   Plant poles are near the unit circle (|z| ~ 0.99)

% Design 1: Moderate observer (poles at 0.5, 0.4)
obs_poles_slow = [0.5, 0.4];
L_slow = place(A', C', obs_poles_slow)';

% Design 2: Fast observer (poles at 0.1, 0.05)
obs_poles_fast = [0.1, 0.05];
L_fast = place(A', C', obs_poles_fast)';

fprintf('\n---- Observer (Slow) ----\n');
fprintf('Desired poles: '); disp(obs_poles_slow);
fprintf('L = \n'); disp(L_slow);

fprintf('---- Observer (Fast) ----\n');
fprintf('Desired poles: '); disp(obs_poles_fast);
fprintf('L = \n'); disp(L_fast);

%% ---- Simulation ---------------------------------------------------------

T = 150;   % number of steps

% Initial conditions
x0    = [2; -1];    % true initial state
xhat0 = [0;  0];    % observer initial estimate (unknown)

% Input signal: sinusoidal
u = 0.5 * sin(0.2*(0:T-1));

% Storage
x    = zeros(n, T);
y    = zeros(1, T);
xhat_slow = zeros(n, T);
xhat_fast = zeros(n, T);

x(:,1)         = x0;
xhat_slow(:,1) = xhat0;
xhat_fast(:,1) = xhat0;

for k = 1:T-1
    % True system
    x(:,k+1) = A*x(:,k) + B*u(k);
    y(k)     = C*x(:,k);

    % Observer (slow)
    xhat_slow(:,k+1) = A*xhat_slow(:,k) + B*u(k) ...
                        + L_slow*(y(k) - C*xhat_slow(:,k));

    % Observer (fast)
    xhat_fast(:,k+1) = A*xhat_fast(:,k) + B*u(k) ...
                        + L_fast*(y(k) - C*xhat_fast(:,k));
end
y(T) = C*x(:,T);

% Estimation errors
e_slow = x - xhat_slow;
e_fast = x - xhat_fast;

% Time axis
t = (0:T-1) * Ts;

%% ---- Plots --------------------------------------------------------------

figure('Position', [100 100 850 800])

% (1) State x1 (position)
subplot(3, 1, 1)
stairs(t, x(1,:), 'k', 'LineWidth', 2); hold on
stairs(t, xhat_slow(1,:), 'b--', 'LineWidth', 1.5)
stairs(t, xhat_fast(1,:), 'm-.', 'LineWidth', 1.5)
plot(t, y, 'r.', 'MarkerSize', 6)
legend('True x_1', 'Slow observer', 'Fast observer', 'Measurement y', ...
       'Location', 'best')
xlabel('Time [s]')
ylabel('x_1 (position)')
title('State x_1: True vs Observer Estimates')
grid on

% (2) State x2 (velocity) — unmeasured state
subplot(3, 1, 2)
stairs(t, x(2,:), 'k', 'LineWidth', 2); hold on
stairs(t, xhat_slow(2,:), 'b--', 'LineWidth', 1.5)
stairs(t, xhat_fast(2,:), 'm-.', 'LineWidth', 1.5)
legend('True x_2', 'Slow observer', 'Fast observer', ...
       'Location', 'best')
xlabel('Time [s]')
ylabel('x_2 (velocity)')
title('State x_2 (unmeasured): Estimated from y = C x')
grid on

% (3) Estimation error norm
subplot(3, 1, 3)
e_norm_slow = vecnorm(e_slow);
e_norm_fast = vecnorm(e_fast);
stairs(t, e_norm_slow, 'b', 'LineWidth', 1.5); hold on
stairs(t, e_norm_fast, 'm', 'LineWidth', 1.5)
yline(0, 'k:')
legend(sprintf('Slow (poles: %.1f, %.1f)', obs_poles_slow), ...
       sprintf('Fast (poles: %.2f, %.2f)', obs_poles_fast), ...
       'Location', 'best')
xlabel('Time [s]')
ylabel('||e(k)||')
title('Estimation Error Norm: Effect of Observer Pole Placement')
grid on

sgtitle('Discrete-Time Luenberger Observer', 'FontSize', 14, 'FontWeight', 'bold')

%% ---- Pole-Zero Map ------------------------------------------------------

figure('Position', [950 100 450 400])
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 0.5); hold on
plot(real(eig(A)), imag(eig(A)), 'kx', 'MarkerSize', 12, 'LineWidth', 2)
plot(real(obs_poles_slow), imag(obs_poles_slow), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(obs_poles_fast), imag(obs_poles_fast), 'ms', 'MarkerSize', 10, 'LineWidth', 2)
legend('Unit circle', 'Plant poles', ...
       sprintf('Slow obs poles (%.1f, %.1f)', obs_poles_slow), ...
       sprintf('Fast obs poles (%.2f, %.2f)', obs_poles_fast), ...
       'Location', 'best')
xlabel('Real')
ylabel('Imaginary')
title('Observer Pole Locations')
axis equal; grid on
xlim([-1.3, 1.3]); ylim([-1.3, 1.3])
