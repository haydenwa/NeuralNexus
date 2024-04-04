%% Reset Workspace
clear, clc, close all
rng(0);

%% Perform Parameter Sweep
Niter = 1000; % Number of iterations, increase this for smoother plots
Ilb = 290; % Moment of inertia lower bound
Iub = 5000; % Moment of inertia upper bound
I_range = linspace(Ilb, Iub, Niter); % Moment of inertia range
sigma_n = 10e-6; % Noise, default
Tdel = 10; % Time delay, default
use_lqr = true;
use_pd = false;

% Declare arrays
JT_LQR = NaN(1, Niter); % Array to hold cost for LQR controller
JT_PD = NaN(1, Niter); % Array to hold cost for PD controller
i = 1;
for I = I_range % Iterate over the noise range
    JT_LQR(i) = hovering_sim(I, Tdel, sigma_n, use_lqr); % simulate LQR
    JT_PD(i) = hovering_sim(I, Tdel, sigma_n, use_pd); % simulate PD
    i = i+1;
end

%% Plot Results
figure(1)
plot(I_range, JT_PD, 'r--', 'LineWidth', 1.5);
hold on
plot(I_range, JT_LQR, 'b', 'LineWidth', 1.5);
hold off
grid on
xlabel('Moment of Inertia $$I$$', 'Interpreter', 'latex');
ylabel('$$J(T)$$', 'Interpreter', 'latex');
xlim([Ilb, Iub]);
legend('PD', 'LQR');
title('Hovering Cost');