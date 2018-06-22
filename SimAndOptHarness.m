%% Optimization with NLP

% Contributors:
%       Mauro Salazar, ETH Zuerich, mauro.salazarvillalon@gmail.com
%       John Alsterda, Stanford U,  alsterda@stanford.edu
%       Qizhan Tam,    Stanford U,  qtam@stanford.edu

clear all;
% close all; clc


%% Include Dependencies into Path
% comment this if you already have YALMIP in your path somewhere else...
addpath(genpath('..\AA203'));
% addpath(genpath('..\..\MATLAB\YALMIP-master'));


%% Define Parameters
% load Boundary Values and Opt Params
BVs          = loadSolveParameters();   % init and final vals found here
fn           = fieldnames(BVs);
for i = 1:length(fn)
   p.(fn{i}) = BVs.(fn{i});             % load struct BVs into p
end

% load MARTY parameters
vehicle      = loadVehicleMARTY();
fn           = fieldnames(vehicle);
for i = 1:length(fn)
   p.(fn{i}) = vehicle.(fn{i});         % load struct vehicle into p
end

% print problem introduction to terminal
message     = ['Aiming to drift about a left turn with:\n' ...
               '    radius %.1f m\n' ...
               '    speed %.1f m/s       (Ux %.1f & Uy %.1f)\n' ...
               '    beta %.1f deg      (%.1f rad)\n' ...
               '    yaw rate %.1f deg/s (%.1f rad/s)\n\n'];
fprintf(message, p.R_drift, sqrt(p.Ux_f^2 + p.Uy_f^2), p.Ux_f, ...
        p.Uy_f, p.beta_f*180/pi, p.beta_f, p.r_f*180/pi, p.r_f);


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
settings.debug                  = 0;
settings.usex0                  = 0;


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

% sol.t = cumsum([0 sol.variable.dt]);               % use for size(dt) = N
sol.t = 0 : sol.variable.dt : p.N*sol.variable.dt; % use for size(dt) = 1

% cleanup extra variables
clearvars fn copynames i integration message;


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



