function BVs = loadSolveParameters()
%LOADBOUNDARYVALUES
%   Define the initial and final conditions for drift init project

% optimization parameters
% BVs.T           = 2;                      % [sec] Total time to hit target
% BVs.dt           = 0.025;                 % [sec] Time step
BVs.dtmin        = 0.008;                  % [sec] lower lim on dt
BVs.dtmax        = 0.05;                  % [sec] upper lim on dt
% BVs.dtminAve     = 0.01;                  % [sec] keep average dt above this
BVs.nSS          = 1;                     % # of steps in SS constraint
BVs.N            = 200 + BVs.nSS;         % # of time steps

% optimization options
BVs.integration  = 0; % Integration Scheme (0-Euler's, 1-Heun's)
BVs.assign_mode  = 0; % Include guesses from DP?

% Initial conditions
BVs.E_0         = 0;                      % [m]     Initial East position
BVs.N_0         = -20;                    % [m]     Initial North position
BVs.Psi_0       = 0;                      % [rad]   Initial Heading position
BVs.Ux_0        = 10;                     % [m/s]   Initial x speed
BVs.Uy_0        = 0;                      % [m/s]   Initial y speed
BVs.r_0         = 0;                      % [rad/s] Initial yaw rate

% Terminal drift conditions

    % These values pulled from sol_Equilibrium, calc'd by a 1-step IPOPT ~10 May
    % (new dynamics etc may expire these values. Suggest writing a script to
    % calc this fresh before each optimization)
    % Note: these are quite far from experimentally observed states!
    load('sol_Equilibrium.mat');          % eq states and inputs found by IPOPT
    % Or, a desired drift can be calculed by Jon Goh's scripts:
    % R           = 6;                    % [m]     Radius of drift (if chosen)
    % beta        = -30*pi/180;           % [rad]   Drift sideslip (if chosen)
    % eqStates    = calcDriftEqStates(R,beta,vehicle);
BVs.R_drift   =  R;                       % [m]     Pulled from sol_Equilibrium
BVs.E_f       =  0;                       % [m]     Final East position
BVs.N_f       =  0;                       % [m]     Final North position
BVs.Psi_f     = -final.state.beta;        % [rad]   Final Orientation
BVs.Ux_f      =  final.state.Ux;          % [m/s]   Final x speed
BVs.Uy_f      =  final.state.Uy;          % [m/s]   Final y speed
BVs.V_f       =  sqrt(BVs.Ux_f^2+BVs.Uy_f^2); % [m/s] Final speed
BVs.beta_f    =  final.state.beta;        % [rad]   Final sideslip
BVs.r_f       =  final.state.r;           % [rad/s] Final yaw rate
BVs.Tr_f      =  final.input.Tr;          % [Nm]    Final drive torque
BVs.delta_f   =  final.input.delta;       % [rad]   Final steer angle

end

