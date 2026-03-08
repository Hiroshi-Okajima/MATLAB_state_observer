%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Multirate State Observer Design via LMI Optimization
%
% Paper:
%   H. Okajima, Y. Hosoe and T. Hagiwara,
%   "State Observer Under Multi-Rate Sensing Environment and Its Design
%    Using l2-Induced Norm,"
%   IEEE Access, 2023.
%   https://ieeexplore.ieee.org/document/10054014/
%   DOI: 10.1109/access.2023.3249187
%
% Author: Hiroshi Okajima
% Created with assistance from Claude (Anthropic)
%
% Description:
%   This code demonstrates a multirate observer design where different sensors
%   have different sampling rates. Sensor 1 (y1) samples every 2 steps,
%   while Sensor 2 (y2) samples every 6 steps. The observer is designed using
%   lifted system approach to achieve periodic scheduling.
%
%   Sampling pattern (period M = 6):
%     k mod 6 = 0: Both sensors active
%     k mod 6 = 1: No sensor
%     k mod 6 = 2: Sensor 1 only
%     k mod 6 = 3: No sensor
%     k mod 6 = 4: Sensor 1 only
%     k mod 6 = 5: No sensor
%
% Required MATLAB Toolboxes:
%   - Control System Toolbox
%   - Robust Control Toolbox (for LMI solver)
%
% Related:
%   - Blog: https://blog.control-theory.com/entry/multirate-observer-eng
%   - MATLAB File Exchange: https://jp.mathworks.com/matlabcentral/fileexchange/182941
%   - Code Ocean: https://codeocean.com/capsule/3611894/tree/v1
%   - Research page: https://www.control-theory.com/en/multi-rate-system
%
% License:
%   Creative Commons Attribution 4.0 International (CC BY 4.0)
%   http://creativecommons.org/licenses/by/4.0/
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

set(0, 'DefaultAxesLinewidth', 2, 'DefaultLineLineWidth', 2);
set(0, 'defaultAxesFontSize', 14);
set(0, 'defaultAxesFontName', 'arial');
set(0, 'defaultTextFontName', 'arial');

%% Plant model
al1 = 0.2;   % Process noise level
al2 = 0.05;  % Measurement noise level

T = [1 1 -1; 0 1 1; 0 0 1];
A = inv(T) * [0.92 0.1 0.2; 0 0.90 -0.25; 0 0 0.95] * T;
B = [2; 1; 1];
C = [1 -0.5 0; 0 1 1];
D = [1 0; 0 1];

%% Sensor selection matrices (period M = 6)
% y1: every 2 steps (M1 = 2), y2: every 6 steps (M2 = 6)
S0 = [1 0; 0 1];  % k mod 6 = 0: Both sensors
S1 = [0 0; 0 0];  % k mod 6 = 1: No sensor
S2 = [1 0; 0 0];  % k mod 6 = 2: Sensor 1 only
S3 = [0 0; 0 0];  % k mod 6 = 3: No sensor
S4 = [1 0; 0 0];  % k mod 6 = 4: Sensor 1 only
S5 = [0 0; 0 0];  % k mod 6 = 5: No sensor

%% Augmented (lifted) system matrices
cA = [zeros(3) zeros(3) zeros(3) zeros(3) zeros(3) A;
      A zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
      zeros(3) A zeros(3) zeros(3) zeros(3) zeros(3);
      zeros(3) zeros(3) A zeros(3) zeros(3) zeros(3);
      zeros(3) zeros(3) zeros(3) A zeros(3) zeros(3);
      zeros(3) zeros(3) zeros(3) zeros(3) A zeros(3)];

cB = [zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) B;
      B zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1);
      zeros(3,1) B zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1);
      zeros(3,1) zeros(3,1) B zeros(3,1) zeros(3,1) zeros(3,1);
      zeros(3,1) zeros(3,1) zeros(3,1) B zeros(3,1) zeros(3,1);
      zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) B zeros(3,1)];

cC = [S0*C zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3);
      zeros(2,3) S1*C zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3);
      zeros(2,3) zeros(2,3) S2*C zeros(2,3) zeros(2,3) zeros(2,3);
      zeros(2,3) zeros(2,3) zeros(2,3) S3*C zeros(2,3) zeros(2,3);
      zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) S4*C zeros(2,3);
      zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) S5*C];

cD = [S0*D zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2);
      zeros(2,2) S1*D zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2);
      zeros(2,2) zeros(2,2) S2*D zeros(2,2) zeros(2,2) zeros(2,2);
      zeros(2,2) zeros(2,2) zeros(2,2) S3*D zeros(2,2) zeros(2,2);
      zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2) S4*D zeros(2,2);
      zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2) S5*D];

%% LMI-based observer gain design (minimizing l2-induced norm)
fprintf('Setting up LMI...\n');

setlmis([])

[gamma2, n, sgamma2] = lmivar(1, [1 1]);
[P1, n, sP1] = lmivar(1, [3 1]);
[P2, n, sP2] = lmivar(1, [3 1]);
[P3, n, sP3] = lmivar(1, [3 1]);
[P4, n, sP4] = lmivar(1, [3 1]);
[P5, n, sP5] = lmivar(1, [3 1]);
[P6, n, sP6] = lmivar(1, [3 1]);

[P, n, sP] = lmivar(3, [sP6,     zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                         zeros(3), sP1,      zeros(3), zeros(3), zeros(3), zeros(3);
                         zeros(3), zeros(3), sP2,      zeros(3), zeros(3), zeros(3);
                         zeros(3), zeros(3), zeros(3), sP3,      zeros(3), zeros(3);
                         zeros(3), zeros(3), zeros(3), zeros(3), sP4,      zeros(3);
                         zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), sP5]);

[Y1, n, sY1] = lmivar(2, [3 2]);
[Y2, n, sY2] = lmivar(2, [3 2]);
[Y3, n, sY3] = lmivar(2, [3 2]);
[Y4, n, sY4] = lmivar(2, [3 2]);
[Y5, n, sY5] = lmivar(2, [3 2]);
[Y6, n, sY6] = lmivar(2, [3 2]);

[Y, n, sY] = lmivar(3, [zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2), sY6;
                         sY1,         zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2);
                         zeros(3,2), sY2,         zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2);
                         zeros(3,2), zeros(3,2), sY3,         zeros(3,2), zeros(3,2), zeros(3,2);
                         zeros(3,2), zeros(3,2), zeros(3,2), sY4,         zeros(3,2), zeros(3,2);
                         zeros(3,2), zeros(3,2), zeros(3,2), zeros(3,2), sY5,         zeros(3,2)]);

lmi1 = newlmi;
lmiterm([-lmi1 1 1 P], 1, 1)
lmiterm([-lmi1 1 2 P], 1, cA)
lmiterm([-lmi1 1 2 Y], 1, cC)
lmiterm([-lmi1 2 2 P], 1, 1)
lmiterm([-lmi1 2 2 0], -1)
lmiterm([-lmi1 1 3 P], 1, cB)
lmiterm([-lmi1 3 3 gamma2], 1, eye(6, 6))
lmiterm([-lmi1 1 4 Y], 1, cD)
lmiterm([-lmi1 4 4 gamma2], 1, eye(12, 12))

LMIs = getlmis;
c = [1 zeros(1, 72)];

fprintf('Solving LMI...\n');
[copt, xopt] = mincx(LMIs, c);

%% Extract observer gains
P1opt = dec2mat(LMIs, xopt, P1);
Y1opt = dec2mat(LMIs, xopt, Y1);
P2opt = dec2mat(LMIs, xopt, P2);
Y2opt = dec2mat(LMIs, xopt, Y2);
P3opt = dec2mat(LMIs, xopt, P3);
Y3opt = dec2mat(LMIs, xopt, Y3);
P4opt = dec2mat(LMIs, xopt, P4);
Y4opt = dec2mat(LMIs, xopt, Y4);
P5opt = dec2mat(LMIs, xopt, P5);
Y5opt = dec2mat(LMIs, xopt, Y5);
P6opt = dec2mat(LMIs, xopt, P6);
Y6opt = dec2mat(LMIs, xopt, Y6);

L1 = -P1opt \ Y1opt * S0;
L2 = -P2opt \ Y2opt * S1;
L3 = -P3opt \ Y3opt * S2;
L4 = -P4opt \ Y4opt * S3;
L5 = -P5opt \ Y5opt * S4;
L6 = -P6opt \ Y6opt * S5;

gamma_opt = sqrt(copt);
fprintf('Observer gains computed.\n');
fprintf('Optimal l2-induced norm (gamma): %.4f\n', gamma_opt);

%% Simulation
fprintf('Running simulation...\n');

N_sim = 600;
x  = zeros(3, N_sim + 1);   % Estimated state
xp = zeros(3, N_sim + 1);   % True state
yp = zeros(2, N_sim);        % Measured output
t  = 0:N_sim;

% Input signal
a = linspace(0, 2*pi, N_sim + 1);
u_signal = sin(a);

sensor_active = zeros(2, N_sim);  % Sensor activity log

for k = 1:N_sim
    % Noise
    u_noise = al1 * randn;
    y_noise = al2 * [randn; randn];

    % True plant
    xp(:, k+1) = A * xp(:, k) + B * u_signal(k) + B * u_noise;
    yp(:, k)   = C * xp(:, k) + D * y_noise;

    % Multirate observer
    pattern_idx = rem(k, 6);

    switch pattern_idx
        case 0
            sensor_active(:, k) = [1; 1];
            L_k = L1;
        case 1
            sensor_active(:, k) = [0; 0];
            L_k = L2;
        case 2
            sensor_active(:, k) = [1; 0];
            L_k = L3;
        case 3
            sensor_active(:, k) = [0; 0];
            L_k = L4;
        case 4
            sensor_active(:, k) = [1; 0];
            L_k = L5;
        case 5
            sensor_active(:, k) = [0; 0];
            L_k = L6;
    end

    x(:, k+1) = (A - L_k * C) * x(:, k) + B * u_signal(k) + L_k * yp(:, k);
end

fprintf('Simulation completed.\n');

%% Plot results
figure('Position', [100, 100, 1600, 900]);

% --- Left column ---

% States: true vs estimated
subplot('Position', [0.08, 0.68, 0.35, 0.25]);
hold on;
plot(t(1:N_sim), xp(1, 1:N_sim), 'b--', 'LineWidth', 1.5);
plot(t(1:N_sim), xp(2, 1:N_sim), 'r--', 'LineWidth', 1.5);
plot(t(1:N_sim), xp(3, 1:N_sim), 'g--', 'LineWidth', 1.5);
plot(t(1:N_sim), x(1, 1:N_sim),  'b-',  'LineWidth', 1);
plot(t(1:N_sim), x(2, 1:N_sim),  'r-',  'LineWidth', 1);
plot(t(1:N_sim), x(3, 1:N_sim),  'g-',  'LineWidth', 1);
set(gca, 'XTickLabel', []);
ylabel('States x');
title('States: True (dashed) and Estimated (solid)');
xlim([0, N_sim]);
legend('x_1 (true)', 'x_2 (true)', 'x_3 (true)', ...
       'x_1 (est)',  'x_2 (est)',  'x_3 (est)', ...
       'Location', 'best', 'FontSize', 8);
grid on;

% Input signal
subplot('Position', [0.08, 0.48, 0.35, 0.15]);
plot(t(1:N_sim), u_signal(1:N_sim), 'k-', 'LineWidth', 1.5);
set(gca, 'XTickLabel', []);
ylabel('Input u');
title('Input Signal');
xlim([0, N_sim]);
ylim([min(u_signal) - 0.2, max(u_signal) + 0.2]);
grid on;

% Output y1 (zero when not measured)
subplot('Position', [0.08, 0.28, 0.35, 0.15]);
hold on;
y1_display = zeros(1, N_sim);
for k = 1:N_sim
    if sensor_active(1, k) == 1
        y1_display(k) = yp(1, k);
    end
end
plot(t(1:N_sim), y1_display, 'b-', 'LineWidth', 1.5);
plot([0, N_sim], [0, 0], 'k--', 'LineWidth', 0.5);
set(gca, 'XTickLabel', []);
ylabel('y_1 = x_1 - 0.5x_2');
title('Output y_1 (measured every 2 steps)');
xlim([0, N_sim]);
y1_range = max(abs(yp(1, :))) + 1;
ylim([-y1_range, y1_range]);
grid on;

% Output y2 (zero when not measured)
subplot('Position', [0.08, 0.06, 0.35, 0.15]);
hold on;
y2_display = zeros(1, N_sim);
for k = 1:N_sim
    if sensor_active(2, k) == 1
        y2_display(k) = yp(2, k);
    end
end
plot(t(1:N_sim), y2_display, 'r-', 'LineWidth', 1.5);
plot([0, N_sim], [0, 0], 'k--', 'LineWidth', 0.5);
xlabel('Time step k');
ylabel('y_2 = x_2 + x_3');
title('Output y_2 (measured every 6 steps)');
xlim([0, N_sim]);
y2_range = max(abs(yp(2, :))) + 1;
ylim([-y2_range, y2_range]);
grid on;

% --- Right column: Estimation errors ---

% e1
subplot('Position', [0.52, 0.68, 0.43, 0.23]);
hold on;
plot(t(1:N_sim), xp(1, 1:N_sim) - x(1, 1:N_sim), 'b-', 'LineWidth', 1.5);
plot([0, N_sim], [0, 0], 'k--', 'LineWidth', 1);
set(gca, 'XTickLabel', []);
ylabel('e_1');
title('Estimation Error e_1');
xlim([0, N_sim]);
ylim([-5, 5]);
grid on;

% e2
subplot('Position', [0.52, 0.39, 0.43, 0.23]);
hold on;
plot(t(1:N_sim), xp(2, 1:N_sim) - x(2, 1:N_sim), 'r-', 'LineWidth', 1.5);
plot([0, N_sim], [0, 0], 'k--', 'LineWidth', 1);
set(gca, 'XTickLabel', []);
ylabel('e_2');
title('Estimation Error e_2');
xlim([0, N_sim]);
ylim([-5, 5]);
grid on;

% e3
subplot('Position', [0.52, 0.12, 0.43, 0.23]);
hold on;
plot(t(1:N_sim), xp(3, 1:N_sim) - x(3, 1:N_sim), 'g-', 'LineWidth', 1.5);
plot([0, N_sim], [0, 0], 'k--', 'LineWidth', 1);
xlabel('Time step k');
ylabel('e_3');
title('Estimation Error e_3');
xlim([0, N_sim]);
ylim([-5, 5]);
grid on;

% Title annotation
annotation('textbox', [0.02, 0.97, 0.96, 0.025], ...
    'String', sprintf('Multirate State Observer (y_1: every 2 steps, y_2: every 6 steps)  |  \\gamma = %.4f', gamma_opt), ...
    'FontSize', 13, 'FontWeight', 'bold', 'EdgeColor', 'none');

fprintf('\nDone.\n');
fprintf('Optimal l2-induced norm (gamma): %.4f\n', gamma_opt);
