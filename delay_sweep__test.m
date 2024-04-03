%% Reset Workspace
clear, clc, close all
rng(0);

%% Perform Parameter Sweep
Niter = 10; % increase this for smoother plots
Tdellb = 0;
Tdelub = 30;
Tdel_range = linspace(Tdellb, Tdelub, Niter);
I = 2119; % default
sigma_n = 1e-6; % default
use_lqr = true;

% Declare arrays
JT_K = NaN(1, Niter);
JT_PD = NaN(1, Niter);
i = 1;
for Tdel = Tdel_range
    JT_K(i) = hovering_sim(I, Tdel, sigma_n, use_lqr); % simulate LQR
    JT_PD(i) = hovering_sim(I, Tdel, sigma_n, false); % simulate PD
    i = i+1;
end

%% Plot Results
figure(1)
plot(Tdel_range, JT_PD, 'r--', linewidth=1.5);
hold on
plot(Tdel_range, JT_K, 'b', linewidth=1.5);
hold off
grid on
xlabel('Sensory Delay [ms]');
ylabel('$$J(T)$$', interpreter='latex');
legend('PD', 'LQR');
title('Hovering Cost');

