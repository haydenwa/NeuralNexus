clear all; close all;
% Setting Parameters


% Define Other Parameters
simTime = 3; % seconds
deltaT = 10%/1000; % ms, sensory time delay, from Fig 5 in Cheng paper, chosen somewhat arbitrarily from allowable

% Define P(s) parameters
K = 1;
I = 2119; % moment of inertia for magnificent hummingbird pitch
tauP = 69.7%/1000; % I/b, the open-loop time constant in ms
tauC = 40%/1000; % ms, controller time constant from Fig5 as above
Kp = K * tauC;
Ki = K;


K = 100
A = [0 1; 0 1/(I*tauP)]
B = [0; 1/I]
C = [1 0; 0 1]
D = [0; 0]

% Run simulation
sim("BirdModel.slx");

% Extract simulation results
thetaR = ans.yout(:, 3);
time = ans.tout;


% Plot signals
figure;
subplot(2, 1, 1);
plot(time, thetaR, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angle');
title('Theta and ThetaR');
legend('ThetaR');
    
