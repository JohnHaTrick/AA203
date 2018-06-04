function BVs = loadBoundaryValues()
%LOADBOUNDARYVALUES
%   Define the initial and final conditions for drift init project

% Initial conditions
BVs.E_0       = 0;                        % [m]     Initial East position
BVs.N_0       = 0;                        % [m]     Initial North position
BVs.Psi_0     = 0;                        % [rad]   Initial Heading position
BVs.Ux_0      = 10;                       % [m/s]   Initial x speed
BVs.Uy_0      = 0;                        % [m/s]   Initial y speed
BVs.r_0       = 0;                        % [rad/s] Initial yaw rate

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
BVs.R_drift   = R;                        % [m]     Pulled from sol_Equilibrium
BVs.E_f       =  0;                       % [m]     Final East position
BVs.N_f       =  20;                      % [m]     Final North position
BVs.Psi_f     = -final.state.beta;        % [rad]   Final Orientation
BVs.Ux_f      =  final.state.Ux;          % [m/s]   Final x speed
BVs.Uy_f      =  final.state.Uy;          % [m/s]   Final y speed
BVs.V_f       =  sqrt(BVs.Ux_f^2+BVs.Uy_f^2); % [m/s] Final speed
BVs.beta_f    =  final.state.beta;        % [rad]   Final sideslip
BVs.r_f       =  final.state.r;           % [rad/s] Final yaw rate
BVs.Tr_f      =  final.input.Tr;          % [Nm]    Final drive torque
BVs.delta_f   =  final.input.delta;       % [rad]   Final steer angle

end

