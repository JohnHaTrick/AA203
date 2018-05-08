%% Optimization with NLP

% Contributors:
%       Mauro Salazar, ETH Zuerich, mauro.salazarvillalon@gmail.com
%       John Alsterda, Stanford U,  alsterda@stanford.edu
%       Qizhan Tam,    Stanford U,  qtam@stanford.edu

clear all; close all; clc


%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
addpath(genpath('..\..\MATLAB\YALMIP-master'));
addpath('DriftEquilibriumScripts');


%% Define Parameters
% Horizon length
p.T             = 4;                % [sec]   Total time to hit target
p.dt            = 0.05;             % [sec]   Time step
p.N             = round(p.T/p.dt);  %         # of time steps
p.nSS           = 10;               %         # of steps in SS constraint
% Check N*dt = T

% load MARTY parameters
vehicle = loadVehicleMARTY();
fn = fieldnames(vehicle);
for i = 1:length(fn)
   p.(fn{i}) = vehicle.(fn{i});     % load struct vehicle into p
end

% Initial conditions
p.E_0       = 0;                    % [m]     Initial East position
p.N_0       = 0;                    % [m]     Initial North position
p.Psi_0     = 0;                    % [rad]   Initial Heading position
p.Ux_0      = 10;                   % [m/s]   Initial x speed
p.Uy_0      = 0;                    % [m/s]   Initial y speed
p.r_0       = 0;                    % [rad/s] Initial yaw rate

% Terminal drift conditions
R           = 6;                    % [m]     Radius of drift (chosen)
beta        = -30*(pi/180);         % [rad]   Drift sideslip (chosen)
%   calculate driftEq r, V, delta, Fxr
eqStates    =  calcDriftEqStates(R,beta,vehicle);
p.E_f       = 0;                    % [m]     Final East position
p.N_f       = 50;                   % [m]     Final North position
p.Psi_f     = -beta;                % [rad]   Final Orientation
p.Ux_f      = eqStates.V*cos(beta); % [m/s]   Final x speed
p.Uy_f      = eqStates.V*sin(beta); % [m/s]   Final y speed
p.r_f       = eqStates.r;           % [rad/s] Final yaw rate
p.delta_f   = eqStates.delta;
message = ['Aiming to drift about a left turn with:\n    radius %.1f m\n' ...
           '    speed %.1f m/s (Ux %.1f & Uy %.1f)\n    beta %.1f deg (%.1f rad)\n' ...
           '    yaw rate %.1f deg/s (%.1f rad/s)\n\n'];
fprintf(message, R, eqStates.V, p.Ux_f, p.Uy_f, beta*180/pi, beta, eqStates.r*180/pi, eqStates.r);
                         

%% NLP Settings
settings                   = sdpsettings;
settings.solver            = 'ipopt';
settings.ipopt.print_level = 5;
settings.ipopt.tol         = 1e-2;
settings.verbose           = 3;
settings.debug             = 1;


%% Optimization Object
[objective, constraints, variables] = DriftNonlinear(p);


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
t           = p.dt*(0:N);


%% Plot Results
plotNonlinearSoln(t,sol,p,eqStates);
