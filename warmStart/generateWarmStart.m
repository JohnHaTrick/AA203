%% GENERATE WARM START
% work backwards in a dynamic programming strategy to find a rough plan for
% initiating a drift

clear

%% Final conditions:
load('sol_Equilibrium');
% state: [ xE, yN, Psi, Ux, Uy, r ] = [ 0, 0, -beta, Ux_eq, Uy_eq, r_eq ]
x_f = [ 0, 0, -final.state.beta, final.state.Ux, final.state.Uy, final.state.r ];
% x_f = [ 0, 0, 0, final.state.Ux, 0, 0 ];
% input: [ Tr, delta ]
Tr_f = final.input.Tr;
delta_f = final.input.delta;


%% Solution format:
PI(1).x     = x_f;
PI(1).Tr    = Tr_f;                     % optimal Tr for PI(i).x
PI(1).delta = delta_f;                  % optimal delta for PI(i).x
PI(1).next  = 0;                        % stores i in PI(i) which Tr and delta take you


%% parameters
dt    = .15;                              % [sec] time step length
% depth = 3;                              % how many actions will we search?
a     = linspace(1,11,11);                % how many different actions to consider?
                                        % will generate depth^a nodes
vehicle = loadVehicleMARTY();
                            
%% fill PI
i = 1;
while length(PI) < 340000
    for a_try = a                       % get x_prev that would lead to x from each a
        [Tr_try, delta_try] = getAction(a_try, Tr_f, delta_f);
        x_prev      = backwardsEuler(PI(i).x, Tr_try, delta_try, dt, vehicle);
        if abs(x_prev(1))   <  2    && ... % xE
           x_prev(2)        > -21   && ... % yN
           x_prev(3)*180/pi > -10   && ... % Psi floor
           x_prev(3)*180/pi <  50   && ... % Psi ceil
           x_prev(4)        >  5    && ... % Ux  floor
           x_prev(4)        <  13   && ... % Ux  ceil
           x_prev(5)        > -5    && ... % Uy  floor
           x_prev(5)        <  1    && ... % Uy  ceil
           x_prev(6)        > -1    && ... % r   floor
           x_prev(6)        <  1.5         % r   ceil
            j           = length(PI)+1;     % add new entry to PI
            PI(j).x     = x_prev;           
            PI(j).Tr    = Tr_try;           % fill out PI
            PI(j).delta = delta_try;
            PI(j).next  = i;
        end
    end
    i = i+1;
end

%% Find state nearest to x_0
BVs = loadSolveParameters();     % Get thje init state
    x_0      = nan(6,1);        % load into array like PI.x
    x_0(1)   = BVs.E_0;
    x_0(2)   = BVs.N_0;
    x_0(3)   = BVs.Psi_0;
    x_0(4)   = BVs.Ux_0;
    x_0(5)   = BVs.Uy_0;
    x_0(6)   = BVs.r_0;
i_closest    = 0;               % store current best i
norm_closest = inf;             %   and best norm
for i = 1:length(PI)
    norm_try = norm( x_0' - PI(i).x, 2 );
%     norm_try = norm( x_0(1:2)' - PI(i).x(1:2), 2 );
%     norm_try = norm( x_0' - PI(i).x, 2 ) + norm( x_0(2)' - PI(i).x(2), 2 );
    if norm_try < norm_closest      % find closest i
        i_closest = i;
        norm_closest = norm_try;
    end
end

%% plot search results
X = vertcat(PI.x);
figure();
subplot(2,2,1); hold on; axis equal;
    xlabel('xE'); ylabel('yN')
    rectangle('Position',[PI(1).x(1)-2*BVs.R_drift, PI(1).x(2)-BVs.R_drift, ...
                          2*BVs.R_drift,            2*BVs.R_drift], ...
              'Curvature', [1 1]);
%     for i = 1:length(PI)
%         scatter(PI(i).x(1),PI(i).x(2)) % VERY SLOW!  DO AS ONE CALL
%     end
    scatter(X(:,1),X(:,2))
    scatter(PI(i_closest).x(1),PI(i_closest).x(2), 'LineWidth', 3)
    scatter(BVs.E_0,BVs.N_0, 'LineWidth', 3)
subplot(2,2,2); hold on;
    xlabel('Psi'); ylabel('r')
    scatter(X(:,3),X(:,6))
    scatter(x_f(3),x_f(6), 'LineWidth', 3)
subplot(2,2,3); hold on;
    xlabel('Ux'); ylabel('Uy')
    scatter(X(:,4),X(:,5))
    scatter(x_f(4),x_f(5), 'LineWidth', 3)

%% Store action and state sequence for that initial point
i = i_closest;
% start with closest state and action
    DP.xE    = PI(i).x(1);
    DP.yN    = PI(i).x(2);
    DP.Psi   = PI(i).x(3);
    DP.Ux    = PI(i).x(4);
    DP.Uy    = PI(i).x(5);
    DP.r     = PI(i).x(6);
    DP.Tr    = PI(i).Tr;
    DP.delta = PI(i).delta;
    i              = PI(i).next;
% now step through PI to tack on the rest
while i ~= 0
    DP.xE    = [ DP.xE,    PI(i).x(1)  ];
    DP.yN    = [ DP.yN,    PI(i).x(2)  ];
    DP.Psi   = [ DP.Psi,   PI(i).x(3)  ];
    DP.Ux    = [ DP.Ux,    PI(i).x(4)  ];
    DP.Uy    = [ DP.Uy,    PI(i).x(5)  ];
    DP.r     = [ DP.r,     PI(i).x(6)  ];
    if i~= 1
        DP.Tr    = [ DP.Tr,    PI(i).Tr    ];
        DP.delta = [ DP.delta, PI(i).delta ];
    end
    i              = PI(i).next;
end

%% Interpolare to match expected length for Nonlinear Opt.
% doesn't work for Heun integration (with N+1 inputs)
t_final = (length(DP.xE)-1)*dt;
t_DP    = 0 : dt : t_final;
t_NL    = linspace(0, t_final, BVs.N+1); 
% guess_DP.xE    = interp1(t_DP, DP.xE,    t_NL);
% guess_DP.yN    = interp1(t_DP, DP.yN,    t_NL);
% guess_DP.Psi   = interp1(t_DP, DP.Psi,   t_NL);
% guess_DP.Ux    = interp1(t_DP, DP.Ux,    t_NL);
% guess_DP.Uy    = interp1(t_DP, DP.Uy,    t_NL);
% guess_DP.r     = interp1(t_DP, DP.r,     t_NL);
guess_DP.Tr    = interp1(t_DP(1:end-1), DP.Tr,    t_NL(1:end-1));
guess_DP.delta = interp1(t_DP(1:end-1), DP.delta, t_NL(1:end-1));

% try starting from x_0 instead
guess_DP = forwardEuler(x_0,guess_DP,t_NL,vehicle);

save('parameters/guess_DP.mat','guess_DP');

%% plot results
figure()
subplot(3,2,1); hold on; axis equal;
    title('East-North')
    scatter(BVs.E_0,BVs.N_0, 'LineWidth', 4)
    plot(DP.xE,       DP.yN)
    plot(guess_DP.xE, guess_DP.yN)
subplot(3,2,2); hold on;
    title('speeds')
    plot(t_NL,guess_DP.Ux)
    plot(t_NL,guess_DP.Uy)
    legend('Ux','Uy')
subplot(3,2,3); hold on;
    title('Psi')
    plot(t_NL,guess_DP.Psi)
subplot(3,2,4); hold on;
    title('r')
    plot(t_NL,guess_DP.r)
subplot(3,2,5); hold on;
    title('Tr')
    stairs(t_DP(1:end-1), DP.Tr)
    stairs(t_NL(1:end-1), guess_DP.Tr)
subplot(3,2,6); hold on;
    title('delta')
    stairs(t_DP(1:end-1), DP.delta)
    stairs(t_NL(1:end-1), guess_DP.delta)

    
    
% End script:
%% Helper Functions
                    
% get an action set from a
function [Tr, delta] = getAction(a, Tr_f, delta_f)
    
    switch a                            
        case 1                  % each combination of +/- Tr and delta
            Tr    =  Tr_f;
            delta =  2*delta_f;
        case 2
            Tr    =  Tr_f;
            delta = -2*delta_f;
        case 3
            Tr    =  Tr_f;
            delta =  0;
        case 4
            Tr    = -Tr_f;
            delta =  2*delta_f;
        case 5
            Tr    = -Tr_f;
            delta = -2*delta_f;
        case 6
            Tr    = -Tr_f;
            delta =  0;
        case 7
            Tr    =  0;
            delta =  2*delta_f;
        case 8
            Tr    =  0;
            delta = -2*delta_f;
        case 9
            Tr    =  0;
            delta =  0;
            
        case 10                 
            Tr    =  0;
            delta =  delta_f;
        case 11
            Tr    =  0;
            delta = -delta_f;
%             
%         case 12
%             Tr    = -Tr_f;
%             delta = 2*delta_f;
%         case 13
%             Tr    =  0;
%             delta = -2*delta_f;
%         case 14
%             Tr    = -Tr_f;
%             delta = -2*delta_f;
    end 
end

% get the previous state from action a
function x = backwardsEuler(x, Tr, delta, dt, vehicle)
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
    h  = vehicle.h;
    mu = vehicle.mu;
    
    for i = 1:n
        Fxf     = 0;
        Fxr     = Tr/vehicle.Rwr;
        Fzf     = (m*g*b - h*Fxr)/(a+b);
        Fzr     = (m*g*a + h*Fxr)/(a+b);
        alpha_f = atan2(Uy_f+a*r_f,Ux_f) - delta;
        alpha_r = atan2(Uy_f-b*r_f,Ux_f);
        Fyf     = mu*Fzf*(1-2*exp(vehicle.Wf*alpha_f)/(1+exp(vehicle.Wf*alpha_f)));
        Xi      = sqrt( max(0,(mu*Fzr)^2-Fxr^2) )/mu/Fzr;
        Fyr     = Xi*mu*Fzr*(1-2*exp(vehicle.Wr*alpha_r)/(1+exp(vehicle.Wr*alpha_r)));

        E_f    = E_f   - dt_mini*(-Ux_f*sin(Psi_f) - Uy_f*cos(Psi_f) );
        N_f    = N_f   - dt_mini*( Ux_f*cos(Psi_f) - Uy_f*sin(Psi_f) );
        Psi_f  = Psi_f - dt_mini*( r_f );
        Ux_f   = Ux_f  - dt_mini*( m*r_f*Uy_f + Fxf*cos(delta) - Fyf*sin(delta) + Fxr )/m;
        Uy_f   = Uy_f  - dt_mini*(-m*r_f*Ux_f + Fxf*sin(delta) + Fyf*cos(delta) + Fyr )/m;
        r_f    = r_f   - dt_mini*( a*Fyf*cos(delta) + a*Fxf*sin(delta) - b*Fyr )/Iz;
    end
    
    x(1) = E_f;
    x(2) = N_f;
    x(3) = Psi_f;
    x(4) = Ux_f;
    x(5) = Uy_f;
    x(6) = r_f;
    
end

% generate guesses based only on generated actions and x_0
function guess_DP = forwardEuler(x_0, guess_DP, t_NL, vehicle)

    % dt determined by discretization
    guess_DP.dt     = t_NL(2) - t_NL(1);
    
    % initialize state and lifting variable arrays
    guess_DP.xE     = nan(1,length(guess_DP.Tr)+1);
    guess_DP.yN     = nan(1,length(guess_DP.Tr)+1);
    guess_DP.Psi    = nan(1,length(guess_DP.Tr)+1);
    guess_DP.Ux     = nan(1,length(guess_DP.Tr)+1);
    guess_DP.Uy     = nan(1,length(guess_DP.Tr)+1);
    guess_DP.r      = nan(1,length(guess_DP.Tr)+1);
    guess_DP.alphaF = nan(1,length(guess_DP.Tr));
    guess_DP.alphaR = nan(1,length(guess_DP.Tr));
    guess_DP.Fyf    = nan(1,length(guess_DP.Tr));
    guess_DP.Fyr    = nan(1,length(guess_DP.Tr));
    guess_DP.Fxf    = nan(1,length(guess_DP.Tr));
    guess_DP.Fxr    = nan(1,length(guess_DP.Tr));
    guess_DP.Fzf    = nan(1,length(guess_DP.Tr));
    guess_DP.Fzr    = nan(1,length(guess_DP.Tr));
    guess_DP.Xi     = nan(1,length(guess_DP.Tr));
    
    % initial conditions
    guess_DP.xE(1)  = x_0(1);
    guess_DP.yN(1)  = x_0(2);
    guess_DP.Psi(1) = x_0(3);
    guess_DP.Ux(1)  = x_0(4);
    guess_DP.Uy(1)  = x_0(5);
    guess_DP.r(1)   = x_0(6);

    %Parse params
    m  = vehicle.m;
    Iz = vehicle.Iz;
    g  = vehicle.g;
    a  = vehicle.a;
    b  = vehicle.b;
    h  = vehicle.h;
    mu = vehicle.mu;
    Wf = vehicle.Wf;
    Wr = vehicle.Wr;
    
    for i = 1:length(guess_DP.Tr)
        
        guess_DP.Fxf(i)    = 0;
        guess_DP.Fxr(i)    = guess_DP.Tr(i)/vehicle.Rwr;
        guess_DP.Fzf(i)    = (m*g*b - h*guess_DP.Fxr(i))/(a+b);
        guess_DP.Fzr(i)    = (m*g*a + h*guess_DP.Fxr(i))/(a+b);
        guess_DP.alphaF(i) = atan2(guess_DP.Uy(i)+a*guess_DP.r(i),guess_DP.Ux(i)) ...
                             - guess_DP.delta(i);
        guess_DP.alphaR(i) = atan2(guess_DP.Uy(i)-b*guess_DP.r(i),guess_DP.Ux(i));
        guess_DP.Fyf(i)    = mu*guess_DP.Fzf(i)*( 1-2*exp(Wf*guess_DP.alphaF(i)) ... 
                             / (1+exp(Wf*guess_DP.alphaF(i))) );        
        guess_DP.Xi(i)     = sqrt( max(0,(mu*guess_DP.Fzr(i))^2-guess_DP.Fxr(i)^2) ) ...
                             / mu / guess_DP.Fzr(i);
        guess_DP.Fyr(i)    = guess_DP.Xi(i)*mu*guess_DP.Fzr(i)*( 1-2*exp(Wr*guess_DP.alphaR(i)) ...
                             / (1+exp(Wr*guess_DP.alphaR(i))));

        guess_DP.xE(i+1)   = guess_DP.xE(i)  + guess_DP.dt*(-guess_DP.Ux(i)*sin(guess_DP.Psi(i)) ...
                                                           - guess_DP.Uy(i)*cos(guess_DP.Psi(i)) );
        guess_DP.yN(i+1)   = guess_DP.yN(i)  + guess_DP.dt*( guess_DP.Ux(i)*cos(guess_DP.Psi(i)) ...
                                                           - guess_DP.Uy(i)*sin(guess_DP.Psi(i)) );
        guess_DP.Psi(i+1)  = guess_DP.Psi(i) + guess_DP.dt*( guess_DP.r(i) );
        guess_DP.Ux(i+1)   = guess_DP.Ux(i)  + guess_DP.dt*( m*guess_DP.r(i)*guess_DP.Uy(i) ...
                                                           + guess_DP.Fxf(i)*cos(guess_DP.delta(i)) ...
                                                           - guess_DP.Fyf(i)*sin(guess_DP.delta(i)) ...
                                                           + guess_DP.Fxr(i) )/m;
        guess_DP.Uy(i+1)   = guess_DP.Uy(i)  + guess_DP.dt*(-m*guess_DP.r(i)*guess_DP.Ux(i) ...
                                                           + guess_DP.Fxf(i)*sin(guess_DP.delta(i)) ...
                                                           + guess_DP.Fyf(i)*cos(guess_DP.delta(i)) ...
                                                           + guess_DP.Fyr(i) )/m;
        guess_DP.r(i+1)    = guess_DP.r(i)   + guess_DP.dt*( a*guess_DP.Fyf(i)*cos(guess_DP.delta(i)) ...
                                                           + a*guess_DP.Fxf(i)*sin(guess_DP.delta(i)) ...
                                                           - b*guess_DP.Fyr(i) )/Iz; 
    end
    
end