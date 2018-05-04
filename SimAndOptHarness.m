%% Optimization with NLP


% Contributors:
%       Mauro Salazar, ETH Zuerich, mauro.salazarvillalon@gmail.com,
%       John Alsterda, Stanford U,  alsterda@stanford.edu

clear all; close all; clc
GenerateFig = false;

%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
addpath(genpath('..\..\MATLAB\YALMIP-master'));
addpath('DriftEquilibriumScripts');

%% Define Parameters
% Horizon length
<<<<<<< HEAD:Optimization2018.m
p.dtmin         = 0.01;
p.dtmax         = 0.1;
p.dt            = 0.05; % If doing with fixed time
p.N             = 100;
p.nSS           = 1; % Number of steps in SS
% Check N*dt = T = 5s

% load MARTY parameters
% p = loadVehicleMARTY();

% Main model parameters
p.m         = 1700;     % Mass
p.a         = 1.39;     % dist(CG,front axle)
p.b         = 1.0008;   % dist(CG,rear axle)
p.g         = 9.81;     % Gravitational acceleration
p.CaF       = 75000;    % Cornering stiffness front
p.CaR       = 275000;   % Cornering stiffness rear
p.muF       = 1.15;     % Friction front
p.muR       = 0.85;     % Friction rear
p.mu        = 1;        % Friction average
p.wF        = 12.24;    % Front logit weight
p.wR        = 60.10;    % Rear logit weight
p.wt        = 1.6;      % Track width
p.h         = 0.45;     % CG Height
p.Iz        = 2300;     % Rotational inertia
p.Tmax      = 5400;     % Max torque
p.Tmin      = -15000;   % Min torque
p.Rw        = 0.32;     % Real wheel radius
=======
p.T             = 4;
p.dt            = 0.005;
p.N             = round(p.T/p.dt);
p.nSS           = 10; % Number of steps in SS
% Check N*dt = T = 5s

% load MARTY parameters
vehicle = loadVehicleMARTY();
fn = fieldnames(vehicle);
for i = 1:length(fn)
   p.(fn{i}) = vehicle.(fn{i});
end

% Main model parameters
% p.m         = 1700;     % Mass
% p.a         = 1.39;     % dist(CG,front axle)
% p.b         = 1.0008;   % dist(CG,rear axle)
% p.g         = 9.81;     % Gravitational acceleration
% p.CaF       = 75000;    % Cornering stiffness front
% p.CaR       = 275000;   % Cornering stiffness rear
% p.muF       = 1.15;     % Friction front
% p.muR       = 0.85;     % Friction rear
% p.mu        = 1;        % Friction average
% p.wF        = 12.24;     % Front logit weight
% p.wR        = 60.10;     % Rear logit weight
% p.wt        = 1.6;      % Track width
% p.h         = 0.45;     % CG Height
% p.Iz        = 2300;     % Rotational inertia
% p.Tmax      = 5400;     % Max torque
% p.Tmin      = -15000;   % Min torque
% p.Rw        = 0.32;     % Rear wheel radius
>>>>>>> 015f0cf111db54d04a05849ae6728f05882a7410:SimAndOptHarness.m
p.deltaMax  = 0.671825; % Maximum steering angle
p.Uxmin     = 5;        % Minimum speed

% Initial conditions
p.E_0       = 0;        % Initial East position
p.N_0       = 0;        % Initial North position
p.Psi_0     = 0;        % Initial Heading position
p.Ux_0      = 10;        % Initial x speed
p.Uy_0      = 0;        % Initial y speed
p.r_0       = 0;        % Initial yaw rate

% Terminal drift conditions
R           = 6;                            % [m] radius of drift (chosen)
beta        = -30*(pi/180);                 % [rad] drift sideslip (chosen)
%   calculate driftEq r, V, delta, Fxr
eqStates    =  calcDriftEqStates(R,beta,vehicle);
p.E_f       = 0;                            % Final East position
p.N_f       = 50;                           % Final North position
p.Psi_f     = -beta;                        % Final Orientation
p.Ux_f      = eqStates.V*cos(beta);         % Final x speed
p.Uy_f      = eqStates.V*sin(beta);         % Final y speed
p.r_f       = eqStates.r;                   % Final yaw rate
p.delta_f   = eqStates.delta;
message = ['Aiming to drift about a left turn with:\n    radius %.1f m\n' ...
           '    speed %.1f m/s (Ux %.1f & Uy %.1f)\n    beta %.1f deg (%.1f rad)\n' ...
           '    yaw rate %.1f deg/s (%.1f rad/s)\n\n'];
fprintf(message, R, eqStates.V, p.Ux_f, p.Uy_f, beta*180/pi, beta, eqStates.r*180/pi, eqStates.r);


%% Pacejka Formulation
Fzf  = 1/(p.a+p.b)*p.m*p.b*p.g;
alphaFlimf = atan(3*p.muf*Fzf/p.Caf);

% Front Wheels
Fyf  =  @(alphaF) (-p.Caf*tan(alphaF) + p.Caf^2./(3*p.muf*Fzf).*abs(tan(alphaF)).*tan(alphaF)...
                    - p.Caf^3./(27*p.muf^2*Fzf.^2).*tan(alphaF).^3).*(abs(alphaF)<= alphaFlimf) +...
                    -p.muf*Fzf*sign(alphaF).*(abs(alphaF) > alphaFlimf);
FyfSimpler = @(alphaF,wF) p.muf*Fzf*(1-2*exp(alphaF.*wF)./(1+exp(alphaF.*wF)));
                
Nalpha = 101;
Nw     = 10001;
alphaF = linspace(-pi/6,pi/6,Nalpha);
Wf = linspace(10,17,Nw);

for i = 1:Nw
    DeltaFyfModel(i) = norm(FyfSimpler(alphaF,Wf(i))-Fyf(alphaF));
end

[error, iwFbest] = min(DeltaFyfModel);
wFbest = Wf(iwFbest);

% figure('Name','Force from slip angles','Position',[0 600 400 275])
% plot(alphaF,Fyf(alphaF),'k'); hold on; grid on; box on
% plot(alphaF,FyfSimpler(alphaF,wFbest),'--r');

%% Rear Wheels
Fzr  = 1/(p.a+p.b)*p.m*p.a*p.g;
alphaRlimr = atan(3*p.mur*Fzr/p.Car);
Fyr  =  @(alphaR) (-p.Car*tan(alphaR) + p.Car^2./(3*p.mur*Fzr).*abs(tan(alphaR)).*tan(alphaR)...
                    - p.Car^3./(27*p.mur^2*Fzr.^2).*tan(alphaR).^3).*(abs(alphaR)<= alphaRlimr) +...
                    -p.mur*Fzr*sign(alphaR).*(abs(alphaR) > alphaRlimr);
                
FyrSimpler = @(alphaR,wR) p.mur*Fzr*(1-2*exp(alphaR.*wR)./(1+exp(alphaR.*wR)));
                
Nalpha = 101;
Nw     = 10001;
alphaR = linspace(-pi/6,pi/6,Nalpha);
Wr = linspace(50,70,Nw);

for i = 1:Nw
    DeltaFyrModel(i) = norm(FyrSimpler(alphaR,Wr(i))-Fyr(alphaR));
end

[error, iwFbest] = min(DeltaFyrModel);
wRbest = Wr(iwFbest);

% figure('Name','Force from slip angles','Position',[400 600 400 275])
% plot(alphaR,Fyr(alphaR),'k'); hold on; grid on; box on
% plot(alphaR,FyrSimpler(alphaR,wRbest),'--r');
                         

%% Settings
settings = sdpsettings;
settings.solver = 'ipopt';
settings.ipopt.print_level = 5;
settings.ipopt.tol = 1e-2;
settings.verbose = 3;
settings.debug = 1;

%% Optimization Object
[objective, constraints, variables] = DriftNonlinearFree(p);

%% Solve
diagnostics = optimize(constraints,objective,settings);
% Objective
sol.objective = value(objective);

copynames = fields(variables);
for i = 1:numel(copynames)
    sol.(variables.(copynames{i}).type).(copynames{i}) = value(variables.(copynames{i}));
end

for i = 1:length(constraints)
    sol.dual.(['c' num2str(i)]) = [ {dual(constraints(i))} {tag(constraints(i))} ];
end

%% Gather Data
% State Variables
xE          = sol.state.xE;
yN          = sol.state.yN;
Psi         = sol.state.Psi;
Ux          = sol.state.Ux;
Uy          = sol.state.Uy;
r           = sol.state.r;

% Input Variables
% Tr          = sol.input.Tr;
Tr          = sol.input.Fxr*p.Rwr;
delta       = sol.input.delta;

% Time
N           = p.N;
% t           = p.dt*(0:N);
t           = cumsum([0 sol.variable.dt]);

%% Plot Results
% State Variables
figure('Name','State Variables','Position',[0 0 400 1000])
subplot(611); hold on; box on
plot(t,xE,'k'); grid on
ylabel('E [m]')
set(gca,'xticklabel',{})
subplot(612); hold on; box on
plot(t,yN,'k'); grid on
ylabel('N [m]')
set(gca,'xticklabel',{})
subplot(613); hold on; box on
plot(t,Psi/pi*180,'k'); grid on
set(gca,'xticklabel',{})
ylabel('\Psi [deg]')
subplot(614); hold on; box on
plot(t,Ux,'k'); grid on
ylabel('U_x [m/s]')
set(gca,'xticklabel',{})
subplot(615); hold on; box on
plot(t,Uy,'k'); grid on
ylabel('U_y [m/s]')
set(gca,'xticklabel',{})
subplot(616); hold on; box on
plot(t,r,'k'); grid on
ylabel('r [rad/s]')
xlabel('t [s]')

% Input Variables
figure('Name','Input Variables','Position',[400 0 400 600])
subplot(211); hold on; box on
stairs(t(1:N),Tr,'k'); grid on
ylabel('T_r [Nm]')
ylim([p.Tmin, p.Tmax])
set(gca,'xticklabel',{})
subplot(212); hold on; box on
stairs(t(1:N),delta/pi*180,'k'); grid on
ylabel('\delta [deg]')
xlabel('t [s]')
ylim([-p.deltaMax*180/pi, p.deltaMax*180/pi])

% Trajectory
figure('Name','Trajectory E-N','Position',[800 400 400 400])
plot(xE,yN,'k'); grid on; box on
axis equal
xlabel('E [m]')
ylabel('N [m]')

if GenerateFig
    fileName = 'ICE2018';
    mlf2pdf(gcf,fileName)
    system(strcat('pdfcrop --noverbose ./',fileName,'.pdf ./', fileName,'.pdf'));
end