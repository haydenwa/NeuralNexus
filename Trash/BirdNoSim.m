clear all; close all;
% Setting Parameters


% Define Other Parameters
simTime = 3; % seconds
deltaT = 10%/1000; % ms, sensory time delay, from Fig 5 in Cheng paper, chosen somewhat arbitrarily from allowable

% Define P(s) parameters
I = 2119; % moment of inertia for magnificent hummingbird pitch
tauP = 69.7/1000; % I/b, the open-loop time constant in ms
tauC = 40/1000; % ms, controller time constant from Fig5 as above

% Define ranges for Kp and Ki values to explore
Kp_range = linspace(0, 10, 10); % Adjust as needed
Ki_range = linspace(0, 10, 10); % Adjust as needed

% Initialize variables to store best gains and performance
best_Kp = 0;
best_Ki = 0;
best_performance = Inf;

% Loop over Kp and Ki values
for Kp = Kp_range
    for Ki = Ki_range
        % Define the state-space model with the current gains
        A = [0, 1; 0, -1/(I*tauP)];
        B = [0; 1/I];
        C = [1, 0; 0, 1];
        D = [0; 0];
        
        % Define the feedback gains
        K = [Ki; Kp];
        
        % Simulate the system with feedback control
        sys = ss(A, B, C, D);
        sys_with_feedback = feedback(sys, K(2), K(1), 1);
        
        % Simulate the response to a step input
        t = 0:0.01:10;
        r = ones(size(t)); % Step input
        [y, ~, ~] = lsim(sys_with_feedback, r, t);
        
        % Evaluate performance (you may need to adjust this based on your criteria)
        overshoot = max(y) - 1; % Assuming a step input of 1
        settling_time = t(find(y > 0.98, 1, 'last')) - t(find(y > 1.02, 1, 'first'));
        performance = overshoot + settling_time;
        
        % Update best gains if the current performance is better
        if performance < best_performance
            best_performance = performance;
            best_Kp = Kp;
            best_Ki = Ki;
        end
    end
end

% Display the best gains found
disp(['Best Kp: ', num2str(best_Kp)]);
disp(['Best Ki: ', num2str(best_Ki)]);