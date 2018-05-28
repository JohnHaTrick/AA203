%% Optimization with NLP

% Contributors:
%       Mauro Salazar, ETH Zuerich, mauro.salazarvillalon@gmail.com
%       John Alsterda, Stanford U,  alsterda@stanford.edu
%       Qizhan Tam,    Stanford U,  qtam@stanford.edu

clear all; close all; clc


%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
addpath(genpath('..\AA203'));
% addpath(genpath('..\..\MATLAB\YALMIP-master'));


%% Integration Scheme (Euler's vs Trapz)
integration = 0; %0-Euler's, 1-Heun's


%% Define Parameters
% Horizon length
% p.T         = 2;                        % [sec] Total time to hit target
% p.dt        = 0.025;                    % [sec] Time step
p.dtmin     = 0.01;                     % [sec] lower lim on dt
p.dtmax     = 0.05;                     % [sec] upper lim on dt
% p.dtminAve  = 0.01;                     % [sec] keep average dt above this
p.nSS       = 2;                        % # of steps in SS constraint
p.N         = 150 + p.nSS;              % # of time steps

% load MARTY parameters
vehicle     = loadVehicleMARTY();
fn          = fieldnames(vehicle);
for i = 1:length(fn)
   p.(fn{i}) = vehicle.(fn{i});         % load struct vehicle into p
end

% Initial conditions
p.E_0       = 0;                        % [m]     Initial East position
p.N_0       = 0;                        % [m]     Initial North position
p.Psi_0     = 0;                        % [rad]   Initial Heading position
p.Ux_0      = 10;                       % [m/s]   Initial x speed
p.Uy_0      = 0;                        % [m/s]   Initial y speed
p.r_0       = 0;                        % [rad/s] Initial yaw rate

% Terminal drift conditions
    % Use eq drift states calculated by a 1-step IPOPT ~10 May
    % (new dynamics etc may expire these values. Suggest writing a script to
    % calc this fresh before each optimization)
    % Note: these are quite far from experimentally observed states!
    load('sol_Equilibrium.mat');        % eq states and inputs found by IPOPT
    % Or, a desired drift can be calculed by Jon Goh's scripts:
    % R           = 6;                    % [m]     Radius of drift (if chosen)
    % beta        = -30*pi/180;           % [rad]   Drift sideslip (if chosen)
    % eqStates    = calcDriftEqStates(R,beta,vehicle);
p.E_f       =  0;                       % [m]     Final East position
p.N_f       =  20;                      % [m]     Final North position
p.Psi_f     = -final.state.beta;        % [rad]   Final Orientation
p.Ux_f      =  final.state.Ux;          % [m/s]   Final x speed
p.Uy_f      =  final.state.Uy;          % [m/s]   Final y speed
p.V_f       =  sqrt(p.Ux_f^2+p.Uy_f^2); % [m/s]    Final speed
p.beta_f    =  final.state.beta;        % [rad]   Final sideslip
p.r_f       =  final.state.r;           % [rad/s] Final yaw rate
p.Tr_f      =  final.input.Tr;          % [Nm]    Final drive torque
p.delta_f   =  final.input.delta;       % [rad]   Final steer angle
message     = ['Aiming to drift about a left turn with:\n' ...
               '    radius %.1f m\n' ...
               '    speed %.1f m/s       (Ux %.1f & Uy %.1f)\n' ...
               '    beta %.1f deg      (%.1f rad)\n' ...
               '    yaw rate %.1f deg/s (%.1f rad/s)\n\n'];
fprintf(message, R, sqrt(p.Ux_f^2 + p.Uy_f^2), p.Ux_f, p.Uy_f, ...
        final.state.beta*180/pi, final.state.beta, p.r_f*180/pi, p.r_f);


%% NLP Settings
settings                        = sdpsettings;
settings.solver                 = 'ipopt';
settings.ipopt.print_level      = 5;    % default = 5
settings.ipopt.tol              = 1e-3; % default = 1e-8
settings.ipopt.dual_inf_tol     = 1;    % default = 1
settings.ipopt.constr_viol_tol  = 1e-2; % default = 1e-4
settings.ipopt.compl_inf_tol    = 1e-4; % default = 1e-4
settings.ipopt.acceptable_tol   = 1e-6; % default = 1e-6
settings.ipopt.acceptable_iter  = 10;   % default = 15
    % tol info: https://www.coin-or.org/Ipopt/documentation/node42.html#SECTION000112010000000000000
settings.verbose                = 3;
settings.debug                  = 1;
settings.usex0 = 1;

%% Optimization Object
[objective, constraints, variables] = DriftNonlinear(p,integration);


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

sol.t = cumsum([0 sol.variable.dt]);
p.R   = R;

% cleanup extra variables
clearvars fn copynames i integration message R;


%% Print Results
fprintf('\ndUx/dt = %f\n',(sol.state.Ux(end)-sol.state.Ux(end-1))/(sol.t(end)-sol.t(end-1)));
fprintf('dUy/dt = %f\n',(sol.state.Uy(end)-sol.state.Uy(end-1))/(sol.t(end)-sol.t(end-1)));
fprintf('dr/dt = %f\n',(sol.state.r(end)-sol.state.r(end-1))/(sol.t(end)-sol.t(end-1)));
fprintf('\n((xE-E_f)/10)^2 = %f \n',((sol.state.xE(end)-p.E_f)/10)^2);
fprintf('((yN-N_f)/10)^2 = %f \n',((sol.state.xE(end)-p.N_f)/10)^2);
fprintf('((Psi-Psi_f)/10)^2 = %f \n',((sol.state.Psi(end)-p.Psi_f))^2);
fprintf('sum(abs(Tr))/N = %f \n',sum(abs(sol.input.Tr))/p.N);
fprintf('100*sum(diff(delta).^2)/N = %f \n',100*sum(diff(sol.input.delta).^2)/p.N);
fprintf('sum(dt)/N = %f \n',sum(diff(sol.t))/p.N);


%% Extend results to complete the drift
% Use the final SS inputs and velocity states to evolve for ~3/4 circle
sol = extendDrift(sol,p);


%% Plot Results
% eqStates       = final.state;
% eqStates.Tr    = final.input.Tr;
% eqStates.delta = final.input.delta;
% eqStates.beta  = final.state.beta;
% eqStates.V     = sqrt(eqStates.Ux^2 + eqStates.Uy^2);
plotNonlinearSoln(sol,p);


%% Interpolate results to regular time interval
% possibly add more dirfting states
interpResults = interpolateResults(sol,p);



