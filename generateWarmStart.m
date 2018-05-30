%% GENERATE WARM START
% work backwards in a dynamic programming strategy to find a rough plan for
% initiating a drift

clear

%% Final conditions:
load('sol_Equilibrium');
% state: [ xE, yN, Psi, Ux, Uy, r ] = [ 0, 0, -beta, Ux_eq, Uy_eq, r_eq ]
x_f = [ 0, 0, -final.state.beta, final.state.Ux, final.state.Uy, final.state.r ];
% input: [ Tr, delta ]
Tr_f = final.input.Tr;
delta_f = final.input.delta;


%% Solution format:
PI(1).x     = x_f;
PI(1).Tr    = Tr_f;                     % optimal Tr for PI(i).x
PI(1).delta = delta_f;                  % optimal delta for PI(i).x
PI(1).next  = 0;                        % stores i in PI(i) which Tr and delta take you


%% parameters
dt    = .2;                              % [sec] time step length
% depth = 3;                              % how many actions will we search?
a     = linspace(1,9,9);                % how many different actions to consider?
                                        % will generate depth^a nodes
vehicle = loadVehicleMARTY();
                            
%% fill PI
i = 1;
while length(PI) < 5000
    for a_try = a                       % get x_prev that would lead to x from each a
        [Tr_try, delta_try] = getAction(a_try, Tr_f, delta_f);
        x_prev      = backwardsEuler(PI(i).x, Tr_try, delta_try, vehicle, dt);
        j           = length(PI)+1;     % add new entry to PI
        PI(j).x     = x_prev;           
        PI(j).Tr    = Tr_try;           % fill out PI
        PI(j).delta = delta_try;
        PI(j).next  = i;
    end
    i = i+1;
end

%% plot results
R = 5.2377;
figure(); hold on; axis equal;
rectangle('Position',[PI(1).x(1)-2*R, PI(1).x(2)-R, 2*R, 2*R], 'Curvature', [1 1])
for i = 1:length(PI)
    scatter(PI(i).x(1),PI(i).x(2)) % VERY SLOW!  DO AS ONE CALL
end

                            
%% get an action set from a
function [Tr, delta] = getAction(a, Tr_f, delta_f)
    
    switch a                            % each combination of +/- Tr and delta
        case 1
            Tr    =  Tr_f;
            delta =  delta_f;
        case 2
            Tr    =  Tr_f;
            delta = -delta_f;
        case 3
            Tr    =  Tr_f;
            delta =  0;
        case 4
            Tr    = -Tr_f;
            delta =  delta_f;
        case 5
            Tr    = -Tr_f;
            delta = -delta_f;
        case 6
            Tr    = -Tr_f;
            delta =  0;
        case 7
            Tr    =  0;
            delta =  delta_f;
        case 8
            Tr    =  0;
            delta = -delta_f;
        case 9
            Tr    =  0;
            delta =  0;
    end 
end

%% get the previous state from action a
function x = backwardsEuler(x, Tr, delta, vehicle, dt)
    dt_mini = .01;
    n = dt/dt_mini;

    %Parse states and actions
    E_f   = x(1);   N_f  = x(2);
    Psi_f = x(3);   Ux_f = x(4);
    Uy_f  = x(5);   r_f  = x(6);
    
    %Parse params
    m  = vehicle.m;
    Iz = vehicle.Iz;
    g  = vehicle.g;
    a  = vehicle.a;
    b  = vehicle.b;
    mu = vehicle.mu;
    
    for i = 1:n
        alpha_f = atan2(Uy_f+a*r_f,Ux_f) - delta;
        alpha_r = atan2(Uy_f-b*r_f,Ux_f);
        Fzf     = m*b*g/(a+b); % static
        Fzr     = m*a*g/(a+b);
        Fxf     = 0;
        Fxr     = Tr/vehicle.Rwr;
        Fyf     = mu*Fzf*(1-2*exp(vehicle.Wf*alpha_f)/(1+exp(vehicle.Wf*alpha_f)));
        Xi      = sqrt((mu*Fzr)^2-Fxr^2)/mu/Fzr;
        Fyr     = Xi*mu*Fzr*(1-2*exp(vehicle.Wr*alpha_r)/(1+exp(vehicle.Wr*alpha_r)));

        E_f    = E_f   - dt_mini*(-Ux_f*sin(Psi_f) - Uy_f*cos(Psi_f) );
        N_f    = N_f   - dt_mini*( Ux_f*cos(Psi_f) - Uy_f*sin(Psi_f) );
        Psi_f    = Psi_f - dt_mini*( r_f );
        Ux_f    = Ux_f  - dt_mini*( m*r_f*Uy_f + Fxf*cos(delta) - Fyf*sin(delta) + Fxr )/m;
        Uy_f    = Uy_f  - dt_mini*(-m*r_f*Ux_f + Fxf*sin(delta) + Fyf*cos(delta) + Fyr )/m;
        r_f    = r_f   - dt_mini*( a*Fyf*cos(delta) + a*Fxf*sin(delta) - b*Fyr )/Iz;
    end
    
    x(1) = E_f;
    x(2) = N_f;
    x(3) = Psi_f;
    x(4) = Ux_f;
    x(5) = Uy_f;
    x(6) = r_f;
    
end

