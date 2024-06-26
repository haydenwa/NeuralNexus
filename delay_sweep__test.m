%% Reset Workspace
clear, clc, close all
rng(0);

%% Perform Parameter Sweep
Niter = 1000; % Number of iterations, increase this for smoother plots
Tdellb = 0; % Time delay lower bound
Tdelub = 30; % Time delay upper bound
Tdel_range = linspace(Tdellb, Tdelub, Niter); % Time delay range
I = 2119; % Moment of inertia, default
sigma_n = 1e-6; % Noise, default
use_lqr = true;
use_pd = false;

% Declare arrays
JT_LQR = NaN(1, Niter); % Array to hold cost for LQR controller
JT_PD = NaN(1, Niter); % Array to hold cost for PD controller
i = 1;
for Tdel = Tdel_range % Iterate over the time delay range
    JT_LQR(i) = hovering_sim(I, Tdel, sigma_n, use_lqr); % simulate LQR
    JT_PD(i) = hovering_sim(I, Tdel, sigma_n, use_pd); % simulate PD
    i = i+1;
end

%% Plot Results
figure(1)
plot(Tdel_range, JT_PD, 'r--', linewidth=1.5);
hold on
plot(Tdel_range, JT_LQR, 'b', linewidth=1.5);
hold off
grid on
xlabel('Sensory Delay [ms]', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Total Cost $$J(T)$$', 'Interpreter', 'latex', 'FontSize', 15);
xlim([Tdellb, Tdelub]);
legend('PD', 'LQR');
title('Hovering Cost');

