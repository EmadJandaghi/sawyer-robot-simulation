% Test script to simulate and plot joint torques over time
clear; clc; close all;

% Time vector
t = 0:0.01:5; % 0 to 5 seconds, 10ms steps

% Custom initial conditions
A = [1, 0.8, 0.6, 0.4, 0.2, 0.1, 0]'; % Amplitudes (rad)
phi = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 0]'; % Phase shifts (rad)

% Preallocate torque array
nL = 7;
tau = zeros(nL, length(t));

% Compute torques over time
for i = 1:length(t)
    state = fcn(t(i), A, phi); % Get state at time t(i)
    tau(:, i) = Inverse_dyn(state); % Compute torques
end

% Plot torques
figure('Name', 'Joint Torques Over Time', 'NumberTitle', 'off');
hold on;
colors = lines(nL); % MATLAB color palette
for j = 1:nL
    plot(t, tau(j, :), 'LineWidth', 1.5, 'Color', colors(j, :), ...
         'DisplayName', sprintf('Joint %d', j));
end
hold off;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Joint Torques for Sinusoidal Motion');
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);