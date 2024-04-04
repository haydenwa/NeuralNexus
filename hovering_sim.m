function JT = hovering_sim(I, Tdel, sigma, use_lqr)

% Define plant
tp = 69.7; % [ms]
A = [0, 1; 0, 1/(I*tp)];
B = [0; 1/I];
[nx, nu] = size(B);
C = eye(nx);
D = zeros(1, nu);
G = ss(A,B,C,D);

% Define PD controller
tc = 40; % PD controller time constant [ms]
k = 1;
kp = k;
kd = k*tc;
Kpd = -[kp, kd];

% Define cost matrices
Q = 1e-4*eye(nx);
R = 1e-3*eye(nu);

% Define LQR controller
Klqr = -lqr(A,B,Q,R);

% Select controller
if use_lqr
    K = Klqr;
else
    K = Kpd;
end

% Define time delay (Padé approximation)
ord = 2;
s = tf('s');
delay = exp(-Tdel*s);

DelayBlkDiag = [];
for i = 1:nx
    DelayBlkDiag = blkdiag(DelayBlkDiag, delay);
end

% Generate Padé approximation model
P = ss(pade(DelayBlkDiag, ord));
[Ap,Bp,Cp,Dp] = ssdata(P);
[nv, ny] = size(Bp);

% Generate closed-loop model
Aaug = [A+B*K*Dp*C, B*K*Cp;
        Bp*C,       Ap];
Baug = [-B*K,           B,             B*K*Dp;
         zeros(nv, ny), zeros(nv, nu), Bp     ];
Caug = [C,    zeros(ny,nv);
        Dp*C, Cp           ];
Daug = [zeros(ny, ny+nu+ny);
        zeros(ny, ny+nu)   , Dp];
Tcl = ss(Aaug, Baug, Caug, Daug);

% Specify simulation time
Ts = 1e-3; % Simulation sample time, ms
Tf = 1000; % Simulation end time, ms
tspan = 0:Ts:Tf; % List of sample times, ms 
Nt = length(tspan); % Number of samples

% Define reference command
r = [0.1*ones(Nt, 1), zeros(Nt, 1)];

% Generate disturbance and noise
d = zeros(Nt, nu); % No disturbance (for now)
if sigma == 0
    n = zeros(Nt, ny);
else
    sigma_n = sigma*eye(ny);
    RR = chol(sigma_n);
    n = randn(Nt, ny)*RR;
end

% Simulate closed-loop step response
Y = lsim(Tcl, [r, d, n], tspan);
x = Y(:,1:2)';
xm = Y(:,3:4)';
u = -K*(r' - xm);

% Compute final cost J(T)
[~, J] = cost(x,u,Q,R);
JT = J(:,end);

end

