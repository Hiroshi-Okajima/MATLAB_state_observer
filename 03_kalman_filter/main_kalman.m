%% main_kalman.m
%  Kalman Filter Demonstration: Time-Varying and Steady-State
%
%  This script demonstrates:
%    1. Time-varying Kalman filter with convergence of error covariance P(k)
%    2. Steady-state Kalman filter using the converged gain
%    3. Comparison of both estimators
%
%  System model:
%    x(k+1) = A*x(k) + B*u(k) + w(k),   w(k) ~ N(0, Q)
%    y(k)   = C*x(k) + v(k),             v(k) ~ N(0, R)
%
%  Repository: MATLAB_state_observer/03_kalman_filter/
%  Blog: https://blog.control-theory.com/entry/kalman-filter
%  Hub:  https://blog.control-theory.com/entry/state-observer-estimation
%
%  Author: Hiroshi Okajima, Kumamoto University
%  -----------------------------------------------------------------------

clear; clc; close all;

%% ---- Parameters --------------------------------------------------------

% Plant
A = 0.9;
B = 0.5;
C = 1;

% Noise covariances
Q = 0.1;   % process noise
R = 1;     % measurement noise

% Simulation length
T = 100;

% Reproducibility
rng(0);

%% ---- True System Simulation --------------------------------------------

x = zeros(1, T);
y = zeros(1, T);
u = sin(0.1*(1:T));

x(1) = 5;   % initial state

for k = 1:T-1
    w = sqrt(Q) * randn;
    x(k+1) = A*x(k) + B*u(k) + w;
end

for k = 1:T
    v = sqrt(R) * randn;
    y(k) = C*x(k) + v;
end

%% ---- Time-Varying Kalman Filter ----------------------------------------
%  Shows convergence of P(k) and K(k) to steady-state values.

P = 10;                    % initial error covariance (large = uncertain)
P_hist = zeros(1, T);
K_hist = zeros(1, T);

xhat_tv = zeros(1, T);
xhat_tv(1) = 0;           % initial estimate (unknown)

for k = 1:T-1
    % Kalman gain
    K = A*P*C' / (C*P*C' + R);

    % State estimate update (current-type estimator)
    xhat_tv(k+1) = A*xhat_tv(k) + B*u(k) + K*(y(k) - C*xhat_tv(k));

    % Error covariance update (Riccati recursion)
    P = A*P*A' + Q - A*P*C' / (C*P*C' + R) * C*P*A';

    % Record history
    P_hist(k) = P;
    K_hist(k) = K;
end

% Record final step
P_hist(T) = P;
K_hist(T) = A*P*C' / (C*P*C' + R);

%% ---- Steady-State Kalman Filter ----------------------------------------
%  Compute steady-state gain from the discrete algebraic Riccati equation.
%
%  Note: dare() requires Control System Toolbox.
%  If unavailable, use the converged value: Pss ≈ P_hist(end).

[Pss, ~, ~] = dare(A', C', Q, R);
Kss = A*Pss*C' / (C*Pss*C' + R);

fprintf('Steady-state error covariance Pss = %.6f\n', Pss);
fprintf('Steady-state Kalman gain      Kss = %.6f\n', Kss);
fprintf('Converged P from recursion    P(T) = %.6f\n', P_hist(T));

xhat_ss = zeros(1, T);
xhat_ss(1) = 0;

for k = 1:T-1
    xhat_ss(k+1) = A*xhat_ss(k) + B*u(k) + Kss*(y(k) - C*xhat_ss(k));
end

%% ---- Estimation Error ---------------------------------------------------

err_tv = x - xhat_tv;
err_ss = x - xhat_ss;

fprintf('\nRMSE (Time-Varying KF) = %.4f\n', sqrt(mean(err_tv.^2)));
fprintf('RMSE (Steady-State KF) = %.4f\n', sqrt(mean(err_ss.^2)));

%% ---- Plots --------------------------------------------------------------

figure('Position', [100 100 800 700])

% (1) State estimation result
subplot(3, 1, 1)
plot(1:T, x, 'k', 'LineWidth', 2); hold on
plot(1:T, xhat_tv, 'b--', 'LineWidth', 1.5)
plot(1:T, xhat_ss, 'm-.', 'LineWidth', 1.5)
plot(1:T, y, 'r.', 'MarkerSize', 8)
legend('True state', 'Time-varying KF', 'Steady-state KF', 'Measurement', ...
       'Location', 'best')
xlabel('Time step k')
ylabel('x')
title('State Estimation: Time-Varying vs Steady-State Kalman Filter')
grid on

% (2) Estimation error
subplot(3, 1, 2)
plot(1:T, err_tv, 'b', 'LineWidth', 1.2); hold on
plot(1:T, err_ss, 'm--', 'LineWidth', 1.2)
yline(0, 'k:')
legend('Error (Time-varying KF)', 'Error (Steady-state KF)', ...
       'Location', 'best')
xlabel('Time step k')
ylabel('$x - \hat{x}$', 'Interpreter', 'latex')
title('Estimation Error')
grid on

% (3) Convergence of P(k) and K(k)
subplot(3, 1, 3)
yyaxis left
plot(1:T, P_hist, 'b', 'LineWidth', 1.5)
yline(Pss, 'b--', 'LineWidth', 1)
ylabel('P(k)')

yyaxis right
plot(1:T, K_hist, 'r', 'LineWidth', 1.5)
yline(Kss, 'r--', 'LineWidth', 1)
ylabel('K(k)')

xlabel('Time step k')
title('Convergence of Error Covariance P(k) and Kalman Gain K(k)')
legend('P(k)', 'Steady P_{ss}', 'K(k)', 'Steady K_{ss}', ...
       'Location', 'best')
grid on

sgtitle('Kalman Filter Demonstration', 'FontSize', 14, 'FontWeight', 'bold')
