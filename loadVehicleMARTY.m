function vehicle = loadVehicleMARTY()
<<<<<<< HEAD
%loadVehicleMARTY
=======
%%loadVehicleMARTY
>>>>>>> 015f0cf111db54d04a05849ae6728f05882a7410
%   load MARTY's stats for AA203 drift initiation project

vehicle.name = 'MARTY';

<<<<<<< HEAD
% dimensions & CG location
vehicle.a_m          = ;       % distance from front axle to CG
vehicle.b_m          = ;       % distance from rear axle to CG
vehicle.d_m          = ;       % track width
vehicle.width_m      = ;       % physical width of the car
vehicle.L_m          = vehicle.a_m + vehicle.b_m; // wheelbase
vehicle.aBumper_m    = vehicle.a_m + 0.4953; // distance from CG to front bumper
vehicle.bBumper_m    = vehicle.b_m + 0.5715; // distance from CG to rear bumper
 
    // mass & yaw inertia
    vehicle.m_kg         = 2009;       // mass
    vehicle.Izz_kgm2     = vehicle.m_kg*vehicle.a_m*vehicle.b_m; // yaw moment of inertia, updated by Kegs 2015/12/11 
 
    // roll centers and CG height location
    // h' = h_cg - h_rc
    // h_rc = hrf*b/L + hrr*a/L
    double hrf_m         = 0.1;        // front roll center height
    double hrr_m         = 0.1;        // rear roll center height
    double hprime_m      = 0.37;       // distance from CG to line connecting front and rear roll centers
    vehicle.hcg_m        = hrf_m*vehicle.b_m/vehicle.L_m + hrr_m*vehicle.a_m/vehicle.L_m + hprime_m; // CG height
 
    // wheel properties
    vehicle.Re_m         = 0.30;       // wheel effective radius, for plotting scripts only
 
    // longitudinal force parameters
    // FxDrag = Cd0 + Cd1*Vx + Cd2*Vx^2
    vehicle.Cd0_N        = 241.0;      // rolling resistance, updated by Joe 10/9/2014, .12*2009kg
    vehicle.Cd1_Npmps    = 25.1;       // updated by Joe 10/9/2014, .0125*2009kg
    vehicle.Cd2_Npm2ps2  = 0.0;        // "aero" drag, never investigated
    vehicle.maxFx_N      = 5600;       // max positive longitudinal force, comes from max torque (240Nm) * diff ratio (7) / wheel radius (0.3m), for torque-limited region
    vehicle.maxPower_W   = 75e3;       // X1's RWD electric motor is rated at 75 kW, for power-limited region
 
    // drive and brake distributions [f, r]
    // used to distribute desired Fx command between front and rear axles when calculating plan.
    vehicle.driveDistro[0] = 0.0;      // fraction front wheel drive
    vehicle.driveDistro[1] = 1.0;      // fraction rear wheel drive, sum = 1
    assert((vehicle.driveDistro[0] + vehicle.driveDistro[1]) == 1.0);
    vehicle.brakeDistro[0] = 0.6;      // brake proportioning front wheels
    vehicle.brakeDistro[1] = 0.4;      // brake proportioning rear wheels, sum = 1
    assert((vehicle.brakeDistro[0] + vehicle.brakeDistro[1]) == 1.0);

    // steering properties
    vehicle.steerRatio   = 1;           // HWA to RWA
    vehicle.deltaMax_rad = 18 * DEG_TO_RAD; // limit delta
    vehicle.deltaRateMax_radps = 0.344; // limit on change in delta

    // tire  modeling
    // single friction brush tire model
    vehicle.mu           = 0.91;       // friction coefficient
    vehicle.Caf_Nprad    = 135e3;      // front cornering stiffness
    vehicle.Car_Nprad    = 168e3;      // rear cornering stiffness

    // rotation dimensions for vehicle width compensation in QP
    vehicle.slopeF_mprad = 0;
    vehicle.slopeR_mprad = 0;
    vehicle.dis0F_m      = 0;
    vehicle.dis0R_m      = 0;
    vehicle.psi0_rad     = 0;

=======
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
>>>>>>> 015f0cf111db54d04a05849ae6728f05882a7410

end

