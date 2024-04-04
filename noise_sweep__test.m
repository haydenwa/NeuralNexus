%% Reset Workspace
clear, clc, close all
rng(0);

%% Perform Parameter Sweep
Niter = 1000; % Number of iterations, increase this for smoother plots
sigma_nlb = 0; % Noise Sigma lower bound
sigma_nub = 1e-4; % Noise Sigma upper bound
sigma_n_range = linspace(sigma_nlb, sigma_nub, Niter); % Sigma_n range
I = 2119; % Moment of inertia, default
% sigma_n = 1e-6; % Noise, default
Tdel = 10; % Time delay, default
use_lqr = true;
use_pd = false;

% Declare arrays
JT_LQR = NaN(1, Niter); % Array to hold cost for LQR controller
JT_PD = NaN(1, Niter); % Array to hold cost for PD controller
i = 1;
for sigma_n = sigma_n_range % Iterate over the noise range
    JT_LQR(i) = hovering_sim(I, Tdel, sigma_n, use_lqr); % simulate LQR
    JT_PD(i) = hovering_sim(I, Tdel, sigma_n, use_pd); % simulate PD
    i = i+1;
end

%% Plot Results
figure(1)
plot(sigma_n_range, JT_PD, 'r--', 'LineWidth', 1.5);
hold on
plot(sigma_n_range, JT_LQR, 'b', 'LineWidth', 1.5);
hold off
grid on
xlabel('Noise $$\Sigma_n$$', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('Total Cost $$J(T)$$', 'Interpreter', 'latex', 'FontSize', 20);
xlim([sigma_nlb, sigma_nub]);
legend('PD', 'LQR');
title('Hovering Cost');