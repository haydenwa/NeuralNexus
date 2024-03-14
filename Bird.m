% Define PI controller gains
K = 1;
tauC = 60/1000; % ms, controller time constant from Fig5 as above
Kp = K * tauC;
Ki = K;

% Define P(s) parameters
I = 2119; % moment of inertia for magnificent hummingbird pitch
tauP = 69.7/1000; % I/b, the open-loop time constant in ms

% Define Other Parameters
simTime = 3; % seconds
deltaT = 20/1000; % ms, sensory time delay, from Fig 5 in Cheng paper, chosen somewhat arbitrarily from allowable

% Run simulation
sim("BirdModel.slx");

% Extract simulation results
thetaR = ans.yout(:, 1);
dot_thetaR = ans.yout(:, 2);
theta = ans.yout(:, 3);
dot_theta = ans.yout(:, 4);
time = ans.tout;

% Plot signals
figure;
subplot(2, 1, 1);
plot(time, thetaR, 'b', 'LineWidth', 2);
hold on;
plot(time, theta, 'r--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Angle');
title('Theta and ThetaR');
legend('ThetaR', 'Theta');

subplot(2, 1, 2);
plot(time, dot_thetaR, 'g', 'LineWidth', 2);
hold on;
plot(time, dot_theta, 'm--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Angular Velocity');
title('Dot Theta and Dot ThetaR');
legend('Dot ThetaR', 'Dot Theta');

