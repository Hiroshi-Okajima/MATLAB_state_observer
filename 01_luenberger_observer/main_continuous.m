%% main_continuous.m
%  Continuous-Time Luenberger Observer Demonstration
%
%  Demonstrates:
%    - State observer for a 2nd-order continuous-time LTI system
%    - Observer gain design via pole placement
%    - Convergence of estimation error e(t) = x(t) - xhat(t) -> 0
%
%  System model:
%    dx/dt = A*x + B*u
%    y     = C*x
%
%  Observer:
%    dxhat/dt = A*xhat + B*u + L*(y - C*xhat)
%               = (A - L*C)*xhat + B*u + L*y
%
%  Error dynamics:
%    de/dt = (A - L*C)*e
%    -> e(t) -> 0 if A - L*C is Hurwitz (all eigenvalues in LHP)
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

% 2nd-order system: mass-spring-damper example
%   m*ddot(q) + c*dot(q) + k*q = u
%   state: x = [q; dot(q)]
%
%   Parameters: m = 1, c = 0.5, k = 2

m_val = 1;
c_val = 0.5;
k_val = 2;

A = [0, 1; -k_val/m_val, -c_val/m_val];
B = [0; 1/m_val];
C = [1, 0];   % only position is measured

fprintf('---- Plant ----\n');
fprintf('A = \n'); disp(A);
fprintf('B = \n'); disp(B);
fprintf('C = \n'); disp(C);
fprintf('Plant poles: '); disp(eig(A).');

%% ---- Check Observability ------------------------------------------------

Ob = obsv(A, C);
fprintf('Observability matrix rank = %d (n = %d)\n', rank(Ob), size(A,1));

if rank(Ob) < size(A,1)
    error('System is not observable. Observer design is not possible.');
end

%% ---- Observer Gain Design (Pole Placement) ------------------------------

% Place observer poles faster than plant poles
%   Plant poles: approx -0.25 +/- 1.39j  (natural freq ~ 1.41)
%   Observer poles: chosen 3-5x faster

obs_poles = [-5; -6];

L = place(A', C', obs_poles)';

fprintf('\n---- Observer ----\n');
fprintf('Desired observer poles: '); disp(obs_poles.');
fprintf('L = \n'); disp(L);
fprintf('Eigenvalues of A - L*C: '); disp(eig(A - L*C).');

%% ---- Simulation ---------------------------------------------------------

Tsim = 10;       % simulation time [s]

% Initial conditions
x0    = [2; -1];    % true initial state
xhat0 = [0;  0];    % observer initial estimate (unknown)

% Input signal: step input at t = 0
u_func = @(t) 1.0 * (t >= 0);

% ODE: combined state [x; xhat]
odefun = @(t, z) [
    A*z(1:2) + B*u_func(t);                           % true system
    A*z(3:4) + B*u_func(t) + L*(C*z(1:2) - C*z(3:4))  % observer
];

[t, z] = ode45(odefun, [0, Tsim], [x0; xhat0]);

x_true = z(:, 1:2);
x_est  = z(:, 3:4);
e      = x_true - x_est;

%% ---- Plots --------------------------------------------------------------

figure('Position', [100 100 800 750])

% (1) State x1 (position)
subplot(3, 1, 1)
plot(t, x_true(:,1), 'k', 'LineWidth', 2); hold on
plot(t, x_est(:,1), 'b--', 'LineWidth', 1.5)
legend('True x_1', 'Estimate xhat_1', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_1 (position)')
title('State x_1: True vs Observer Estimate')
grid on

% (2) State x2 (velocity)
subplot(3, 1, 2)
plot(t, x_true(:,2), 'k', 'LineWidth', 2); hold on
plot(t, x_est(:,2), 'b--', 'LineWidth', 1.5)
legend('True x_2', 'Estimate xhat_2', 'Location', 'best')
xlabel('Time [s]')
ylabel('x_2 (velocity)')
title('State x_2: True vs Observer Estimate')
grid on

% (3) Estimation error
subplot(3, 1, 3)
plot(t, e(:,1), 'b', 'LineWidth', 1.5); hold on
plot(t, e(:,2), 'r', 'LineWidth', 1.5)
yline(0, 'k:')
legend('e_1 = x_1 - xhat_1', 'e_2 = x_2 - xhat_2', ...
       'Location', 'best')
xlabel('Time [s]')
ylabel('Estimation error')
title('Convergence of Estimation Error  (poles of A-LC determine decay rate)')
grid on

sgtitle('Continuous-Time Luenberger Observer', 'FontSize', 14, 'FontWeight', 'bold')
