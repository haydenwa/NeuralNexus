%% Reset Workspace
clear, clc, close all
rng(0);

%% Define Plant (Continuous-time)
I = 2119; % [g*mm^2]
tp = 69.7; % [ms]
A = [0, 1; 0, 1/(I*tp)];
B = [0; 1/I];
[nx, nu] = size(B);
C = eye(nx);
D = zeros(1, nu);

G = ss(A,B,C,D)

%% Define PD/LQR Controller
tc = 40; % PD controller time constant [ms]
k = 1;
kp = k;
kd = k*tc;
K = -[kp, kd];

Q = 1e-4*eye(nx);
R = 1e-3*eye(nu);
K = -lqr(A,B,Q,R);

%% Time Delay (Padé Approximation)
Tdel = 10;
ord = 2;
s = tf('s');
delay = exp(-Tdel*s);

DelayBlkDiag = [];
for i = 1:nx
    DelayBlkDiag = blkdiag(DelayBlkDiag, delay);
end

P = ss(pade(DelayBlkDiag, ord))
[Ap,Bp,Cp,Dp] = ssdata(P)
[nv, ny] = size(Bp)

%% Closed-loop Model
Aaug = [A+B*K*Dp*C, B*K*Cp;
        Bp*C,       Ap];
Baug = [-B*K,           B,             B*K*Dp;
         zeros(nv, ny), zeros(nv, nu), Bp     ];
Caug = [C,    zeros(ny,nv);
        Dp*C, Cp           ];
Daug = [zeros(ny, ny+nu+ny);
        zeros(ny, ny+nu)   , Dp];

Tcl = ss(Aaug, Baug, Caug, Daug);

%% Simulate PD Controller
Ts = 1e-3; % ms
tspan = 0:Ts:1000; % ms
Nt = length(tspan);

% Define reference command
step = true;
if step
    r = [0.1*ones(Nt, 1), zeros(Nt, 1)];
else
    r = [0.1*sin(tspan)', zeros(Nt, 1)];
end

% Generate disturbance and noise
d = zeros(Nt, nu);
cov_param = 1e-6;
if cov_param == 0
    n = zeros(Nt, ny);
else
    sigma_n = cov_param*eye(ny);
    RR = chol(sigma_n);
    n = randn(Nt, ny)*RR;
end

% Simulate 
Y = lsim(Tcl, [r, d, n], tspan);
x = Y(:,1:2)';
xm = Y(:,3:4)';
u = -K*(r' - xm);
[c,J] = cost(x,u,Q,R);

%% Plot Results

f = figure(1);
f.Position = [300 250 800 600];

subplot(2,2,1)
plot(tspan, x(1,:), linewidth=1.5);
grid on
xlabel('Time [msec]');
ylabel('Angle [rad]');
title('Pitch Angle');
legend('$$\theta(t)$$', interpreter='latex');

subplot(2,2,2)
plot(tspan, x(2,:), linewidth=1.5);
grid on
xlabel('Time [msec]');
ylabel('Angular Velocity [rad/sec]');
title('Pitch Angular Velocity');
legend('$$\dot{\theta}(t)$$', interpreter='latex');

subplot(2,2,3)
plot(tspan, xm(1,:), linewidth=1.5);
grid on
xlabel('Time [msec]');
ylabel('Angle [rad]');
title('Delayed/Noisy Pitch Angle');
legend('$${\theta}_m(t)$$', interpreter='latex');

subplot(2,2,4)
plot(tspan, xm(2,:), linewidth=1.5);
grid on
xlabel('Time [msec]');
ylabel('Angular Velocity [rad/sec]');
title('Delayed/Noisy Pitch Angular Velocity');
legend('$${\dot{\theta}}_m(t)$$', interpreter='latex');


figure(2)
plot(tspan, u, linewidth=1.5);
grid on
xlabel('Time [msec]');
ylabel('Torque [Nm]');
legend('$$u(t)$$', interpreter='latex')
title('Control Input');

figure(3)
plot(tspan, c, linewidth=1.5);
grid on
xlabel('Time [msec]');
legend('$$c(t)$$', interpreter='latex');
title('Per-step Cost');

figure(4)
plot(tspan, J, linewidth=1.5);
grid on
xlabel('Time [msec]');
legend('$$J(t)$$', interpreter='latex');
title('Total Cost');


