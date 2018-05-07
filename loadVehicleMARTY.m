function vehicle = loadVehicleMARTY()
%%loadVehicleMARTY
%   load MARTY's stats for AA203 drift initiation project

vehicle.name = 'MARTY';

%% dimensions & CG location
vehicle.a          = 1.39;       % distance from front axle to CG
vehicle.b          = 1.0008;     % distance from rear axle to CG
vehicle.wt         = 1.6;        % track width: t? d? right symbol?
% vehicle.width_m      = ?;        % physical width of the car
vehicle.L          = vehicle.a + vehicle.b; % wheelbase
vehicle.h          = .45;        % [m] height of CoM

%% mass & yaw inertia
vehicle.g            = 9.81;        % [m/sec^2] gravity
vehicle.m            = 1700;        % [kg]      mass
vehicle.Fz           = vehicle.m*vehicle.g;
vehicle.Fzr          = vehicle.Fz*vehicle.a/vehicle.L;
vehicle.Fzf          = vehicle.Fz*vehicle.b/vehicle.L;
vehicle.Iz           = 2300;        % 

%     // roll centers and CG height location
%     // h' = h_cg - h_rc
%     // h_rc = hrf*b/L + hrr*a/L
%     double hrf_m         = 0.1;        // front roll center height
%     double hrr_m         = 0.1;        // rear roll center height
%     double hprime_m      = 0.37;       // distance from CG to line connecting front and rear roll centers
%     vehicle.hcg_m        = hrf_m*vehicle.b_m/vehicle.L_m + hrr_m*vehicle.a_m/vehicle.L_m + hprime_m; // CG height


%% Wheel properties
vehicle.Rwf         = 0.29591;    % 205/45R16
vehicle.Rwr         = 0.32;       % wheel effective radius


%% Longitudinal force parameters
vehicle.Tmax = 5400;                % maximum rear torque
vehicle.Tmin = -15000;              % maximum brake torque
% Drag stuff: FxDrag = Cd0 + Cd1*Vx + Cd2*Vx^2
%     vehicle.Cd0_N        = 241.0;      // rolling resistance, updated by Joe 10/9/2014, .12*2009kg
%     vehicle.Cd1_Npmps    = 25.1;       // updated by Joe 10/9/2014, .0125*2009kg
%     vehicle.Cd2_Npm2ps2  = 0.0;        // "aero" drag, never investigated


%% Steering properties
vehicle.deltaMax = 0.671825;        % delta limit
%     vehicle.deltaRateMax_radps = 0.344; // limit on change in delta
%     vehicle.steerRatio   = 1;           // HWA to RWA


%% Tire  modeling
%single friction brush tire model
vehicle.mu           = 1;       % friction coefficient
vehicle.muf          = 1;       % front friction coeff
vehicle.mur          = 1;       % rear friction coeff
vehicle.Caf          = 75000;   % [N/rad] front axel cornering stiffness
vehicle.Car          = 275000;  % [N/rad] front axel cornering stiffness

% logit tire modelling
vehicle.Wf        = 12.24;     % Front logit weight
vehicle.Wr        = 60.10;     % Rear logit weight


%% Drift eq stats
vehicle.ftire.Cx = 3e5;
vehicle.rtire.Cx = 3e5; % what are these? Individual tire Ca?

end

