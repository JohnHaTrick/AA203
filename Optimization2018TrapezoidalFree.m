%% Optimization with NLP
%       Mauro Salazar
%       mauro.salazarvillalon@gmail.com
%       ETH Zuerich

clear all; close all; clc
GenerateFig = false;

%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
% addpath(genpath('..\..\MATLAB\YALMIP-master'));

%% Define Parameters
% Horizon length
p.dtmin         = 0.01;
p.dtmax         = 0.1;
p.N             = 50;
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
p.wF        = 12.24;     % Front logit weight
p.wR        = 60.10;     % Rear logit weight
p.wt        = 1.6;      % Track width
p.h         = 0.45;     % CG Height
p.Iz        = 2300;     % Rotational inertia
p.Tmax      = 5400;     % Max torque
p.Tmin      = -15000;   % Min torque
p.Rw        = 0.32;     % Real wheel radius
p.deltaMax  = 0.671825; % Maximum steering angle

% Initial conditions
p.E_0       = 0;        % Initial East position
p.N_0       = 0;        % Initial North position
p.Psi_0     = 0;        % Initial Heading position
p.Ux_0      = 10;        % Initial x speed
p.Uy_0      = 0;        % Initial y speed
p.r_0       = 0;        % Initial yaw rate

% Terminal conditions
p.E_f       = 0;                            % Final East position
p.N_f       = 50;                           % Final North position
p.Psi_f     = 0.61;                        % Final Orientation
p.Ux_f      = 5.167;      % Final x speed
p.Uy_f      = -3.618;      % Final y speed
p.r_f       = 1.262;                            % Final yaw rat

%% Pacejka Formulation
Fzf  = 1/(p.a+p.b)*p.m*p.b*p.g;
alphaFlimf = atan(3*p.muF*Fzf/p.CaF);

% Front Wheels
Fyf  =  @(alphaF) (-p.CaF*tan(alphaF) + p.CaF^2./(3*p.muF*Fzf).*abs(tan(alphaF)).*tan(alphaF)...
                    - p.CaF^3./(27*p.muF^2*Fzf.^2).*tan(alphaF).^3).*(abs(alphaF)<= alphaFlimf) +...
                    -p.muF*Fzf*sign(alphaF).*(abs(alphaF) > alphaFlimf);
FyfSimpler = @(alphaF,wF) p.muF*Fzf*(1-2*exp(alphaF.*wF)./(1+exp(alphaF.*wF)));
                
Nalpha = 101;
Nw     = 10001;
alphaF = linspace(-pi/6,pi/6,Nalpha);
wF = linspace(10,17,Nw);

for i = 1:Nw
    DeltaFyfModel(i) = norm(FyfSimpler(alphaF,wF(i))-Fyf(alphaF));
end

[error, iwFbest] = min(DeltaFyfModel);
wFbest = wF(iwFbest)

% figure('Name','Force from slip angles','Position',[0 600 400 275])
% plot(alphaF,Fyf(alphaF),'k'); hold on; grid on; box on
% plot(alphaF,FyfSimpler(alphaF,wFbest),'--r');

%% Rear Wheels
Fzr  = 1/(p.a+p.b)*p.m*p.a*p.g;
alphaRlimr = atan(3*p.muR*Fzr/p.CaR);
Fyr  =  @(alphaR) (-p.CaR*tan(alphaR) + p.CaR^2./(3*p.muR*Fzr).*abs(tan(alphaR)).*tan(alphaR)...
                    - p.CaR^3./(27*p.muR^2*Fzr.^2).*tan(alphaR).^3).*(abs(alphaR)<= alphaRlimr) +...
                    -p.muR*Fzr*sign(alphaR).*(abs(alphaR) > alphaRlimr);
                
FyrSimpler = @(alphaR,wR) p.muR*Fzr*(1-2*exp(alphaR.*wR)./(1+exp(alphaR.*wR)));
                
Nalpha = 101;
Nw     = 10001;
alphaR = linspace(-pi/6,pi/6,Nalpha);
wR = linspace(50,70,Nw);

for i = 1:Nw
    DeltaFyrModel(i) = norm(FyrSimpler(alphaR,wR(i))-Fyr(alphaR));
end

[error, iwFbest] = min(DeltaFyrModel);
wRbest = wR(iwFbest)

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
[objective, constraints, variables] = DriftNonlinearTrapezoidalFree(p);

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
Tr          = sol.input.Fxr*p.Rw;
delta       = sol.input.delta;

% Time
N           = p.N;
t           = cumsum([0 sol.variable.dt]);

%% Plot Results
% State Variables
figure('Name','State Variables','Position',[0 0 400 1200])
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
plot(t,Tr,'k'); grid on
ylabel('T_r [Nm]')
ylim([p.Tmin, p.Tmax])
set(gca,'xticklabel',{})
subplot(212); hold on; box on
plot(t,delta/pi*180,'k'); grid on
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