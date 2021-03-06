function [objective, constraints, variables] = DriftNonlinear(p)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

if p.integration == 0 %0-Euler, 1-Heun
    N_var = N;
elseif p.integration == 1
    N_var = N+1;
    Heun = @(x) (x(1:end-1)+x(2:end))/2;
end

%% Initialize Variables
% INPUT VARIABLES
delta   = mdlvar(N_var,1,'input');
Tr      = mdlvar(N_var,10000,'input');
% Fxr     = mdlvar(N,1e4,'input'); % if using Fxr as input

% LIFTING VARIABLES
alphaF  = mdlvar(N,1);
alphaR  = mdlvar(N,1);
Fyf     = mdlvar(N,1e4);
Fxf     = mdlvar(N,1e4);
Fzf     = mdlvar(N,1e4);
Fyr     = mdlvar(N,1e4);
Fxr     = mdlvar(N,1e4);
Fzr     = mdlvar(N,1e4);
Xi      = mdlvar(N,1);
% dt      = mdlvar(N,1e-2);
dt      = mdlvar(1,1e-2);

if p.integration == 1
    alphaF2  = mdlvar(N,1);
    alphaR2  = mdlvar(N,1);
    Fyf2     = mdlvar(N,1e4);
    Fxf2     = mdlvar(N,1e4);
    Fzf2     = mdlvar(N,1e4);
    Fyr2     = mdlvar(N,1e4);
    Fxr2     = mdlvar(N,1e4);
    Fzr2     = mdlvar(N,1e4);
    Xi2      = mdlvar(N,1);
end

% STATE VARIABLES
xE      = mdlvar(N+1,10,'state');
yN      = mdlvar(N+1,10,'state');
Psi     = mdlvar(N+1,1,'state');
Ux      = mdlvar(N+1,10,'state');
Uy      = mdlvar(N+1,4,'state');
r       = mdlvar(N+1,1,'state');

%% Initial Guesses
if assign_mode == 1
    load('guess_DP.mat');
    % INPUT VARIABLES
    assign(delta.variable,  guess_DP.delta/delta.const)
    assign(Tr.variable,     guess_DP.Tr/Tr.const)
    
    % LIFTING VARIABLES
    assign(alphaF.variable, guess_DP.alphaF/alphaF.const)
    assign(alphaR.variable, guess_DP.alphaR/alphaR.const)
    assign(Fyf.variable,    guess_DP.Fyf/Fyf.const)
    assign(Fxf.variable,    guess_DP.Fxf/Fxf.const)
    assign(Fzf.variable,    guess_DP.Fzf/Fzf.const)
    assign(Fyr.variable,    guess_DP.Fyr/Fyr.const)
    assign(Fxr.variable,    guess_DP.Fxr/Fxr.const)
    assign(Fzr.variable,    guess_DP.Fzr/Fzr.const)
    assign(Xi.variable,     guess_DP.Xi/Xi.const)    
    assign(dt.variable,     guess_DP.dt/dt.const)

    % STATE VARIABLES
    assign(xE.variable,     guess_DP.xE/xE.const)
    assign(yN.variable,     guess_DP.yN/yN.const)
    assign(Psi.variable,    guess_DP.Psi/Psi.const)
    assign(Ux.variable,     guess_DP.Ux/Ux.const)
    assign(Uy.variable,     guess_DP.Uy/Uy.const)
    assign(r.variable,      guess_DP.r/r.const)
end

%% Inizialize Constraints
constraints = [];

% Time constraint
constraints = [ constraints
    dt.variable      >= dtmin/dt.const
    dt.variable      <= dtmax/dt.const
%     sum(dt.variable) >= N*dtminAve/dt.const % keep total time above threshold
    ];

% Differential Equations, Euler Integration
if p.integration == 0
    constraints = [ constraints
        ( diff(xE.variable)  == (-   Ux.physical(1:N).*sin(Psi.physical(1:N)) ...
                                   -    Uy.physical(1:N).*cos(Psi.physical(1:N))).*dt.physical/xE.const ): 'East Position'
        ( diff(yN.variable)  == (    Ux.physical(1:N).*cos(Psi.physical(1:N)) ...
                             	  -    Uy.physical(1:N).*sin(Psi.physical(1:N))).*dt.physical/yN.const ): 'North Position'
        ( diff(Psi.variable) ==       r.physical(1:N).*dt.physical/Psi.const ):                         'Orientation'
        ( diff(Ux.variable)  == (   Fxf.physical.*cos(delta.physical) ...
                                 -   Fyf.physical.*sin(delta.physical) ...
                                 +   Fxr.physical                      ...
                                 +   m*r.physical(1:N).*Uy.physical(1:N))/m.*dt.physical/Ux.const ):     'x Velocity'
        ( diff(Uy.variable)  == (   Fxf.physical.*sin(delta.physical) ...
                                +   Fyf.physical.*cos(delta.physical) ...
                                 +   Fyr.physical                      ...
                                -   m*r.physical(1:N).*Ux.physical(1:N))./m.*dt.physical/Uy.const ):     'y Velocity'
        ( diff(r.variable)   == ( a*Fyf.physical.*cos(delta.physical) ...
                                 + a*Fxf.physical.*sin(delta.physical) ...
                                 - b*Fyr.physical)/Iz.*dt.physical/r.const ):                            'Yaw rate'
                   ];
    
elseif p.integration == 1
    
    constraints = [ constraints
        ( diff(xE.variable)  ==  (-   Ux.physical(1:N).*sin(Psi.physical(1:N)) ...
                                   -    Uy.physical(1:N).*cos(Psi.physical(1:N))).*dt.physical/xE.const/2 ...
                                 + (-     Ux.physical(2:N+1).*sin(Psi.physical(2:N+1)) ...
                                   -     Uy.physical(2:N+1).*cos(Psi.physical(2:N+1))).*dt.physical/xE.const/2 ): 'East Position'
        ( diff(yN.variable)  == (     Ux.physical(1:N).*cos(Psi.physical(1:N)) ...
                                  -    Uy.physical(1:N).*sin(Psi.physical(1:N))).*dt.physical/yN.const/2 ...
                                 + (    Ux.physical(2:N+1).*cos(Psi.physical(2:N+1)) ...
                             	  -    Uy.physical(2:N+1).*sin(Psi.physical(2:N+1))).*dt.physical/yN.const/2 ): 'North Position'
        ( diff(Psi.variable) ==      r.physical(1:N).*dt.physical/Psi.const/2 + r.physical(2:N+1).*dt.physical/Psi.const/2):                         'Orientation'
        ( diff(Ux.variable)  == ((     Fxf.physical.*cos(Heun(delta.physical)) ...
                                        -     Fyf.physical.*sin(Heun(delta.physical)) ...
                                        +     Fxr.physical                      ...
                                        +     m*r.physical(1:N).*Uy.physical(1:N) )/m).*dt.physical/Ux.const/2 ...
                                +((     Fxf2.physical.*cos(Heun(delta.physical)) ...
                                        -     Fyf2.physical.*sin(Heun(delta.physical)) ...
                                        +     Fxr2.physical                    ...
                                        +     m*r.physical(2:N+1).*Uy.physical(2:N+1))/m).*dt.physical/Ux.const/2 ):     'x Velocity'
        ( diff(Uy.variable)  == (      Fxf.physical.*sin(Heun(delta.physical)) ...
                                        +   Fyf.physical.*cos(Heun(delta.physical)) ...
                                        +   Fyr.physical                      ...
                                        -   m*r.physical(1:N).*Ux.physical(1:N))./m.*dt.physical/Uy.const/2  ...
                                +(      Fxf2.physical.*sin(Heun(delta.physical)) ...
                                        +   Fyf2.physical.*cos(Heun(delta.physical)) ...
                                        +   Fyr2.physical                      ...
                                        -   m*r.physical(2:N+1).*Ux.physical(2:N+1))./m.*dt.physical/Uy.const/2 ):     'y Velocity'
        ( diff(r.variable)   == (     a*Fyf.physical.*cos(Heun(delta.physical)) ...
                                        +  a*Fxf.physical.*sin(Heun(delta.physical)) ...
                                        -  b*Fyr.physical)/Iz.*dt.physical/r.const/2 ...
                                +(     a*Fyf2.physical.*cos(Heun(delta.physical)) ...
                                        +  a*Fxf2.physical.*sin(Heun(delta.physical)) ...
                                        -  b*Fyr2.physical)/Iz.*dt.physical/r.const/2):                            'Yaw rate'
                    ];
end

% Equilibrium Conditions
constraints = [constraints;
    diff(Ux.variable(end-nSS:end)) == 0
    diff(Uy.variable(end-nSS:end)) == 0
    diff(r.variable(end-nSS:end)) == 0
%     dt.variable(end-nSS+1:end) == dtmax/dt.const
    ];

% State Constraints
constraints = [ constraints
    %     yN.variable >= 0
    %     Psi.variable <= pi/Psi.const
    %     Psi.variable >= -pi/Psi.const
    Ux.variable     >=  0/Ux.const %+ slack.variable
    alphaF.variable <=  pi/2/alphaF.const %+ slack.variable
    alphaF.variable >= -pi/2/alphaF.const %+ slack.variable
    alphaR.variable <=  pi/2/alphaR.const %+ slack.variable
    alphaR.variable >= -pi/2/alphaR.const %+ slack.variable
    ];

if p.integration == 1
    constraints = [ constraints
    alphaF2.variable <=  pi/2/alphaF2.const %+ slack.variable
    alphaF2.variable >= -pi/2/alphaF2.const %+ slack.variable
    alphaR2.variable <=  pi/2/alphaR2.const %+ slack.variable
    alphaR2.variable >= -pi/2/alphaR2.const %+ slack.variable
    ];
end

% Initial Conditions
constraints = [ constraints
    xE.variable(1)      == E_0/xE.const %+ slack.variable
    yN.variable(1)      == N_0/yN.const %+ slack.variable
    Psi.variable(1)     == Psi_0/Psi.const %+ slack.variable
    Ux.variable(1)      == Ux_0/Ux.const
    Uy.variable(1)      == Uy_0/Uy.const %+ slack.variable
    r.variable(1)       == r_0/r.const %+ slack.variable
    ];

% Terminal Conditions
constraints = [ constraints
    Psi.variable(N+1-nSS)     == Psi_f/Psi.const
    Ux.variable(N+1)          == Ux_f/Ux.const
    Uy.variable(N+1)          == Uy_f/Uy.const
    r.variable(N+1)           == r_f/r.const
    Tr.variable(N_var-nSS)    == Tr_f/Tr.const;
    delta.variable(N_var-nSS) == delta_f/delta.const;
    ];

% Input Constraints
constraints = [ constraints
    Tr.variable        <=  Tmax/Tr.const %+ slack.variable
    Tr.variable        >=  Tmin/Tr.const %+ slack.variable
    %     Tr.variable        >= 0                                % no brakes
    delta.variable     <=  deltaMax/delta.const %+ slack.variable
    delta.variable     >= -deltaMax/delta.const %+ slack.variable
%         delta.variable     == 0                                % no steering
%     diff(delta.physical)./dt.physical(1:end-1) <= deltaRateMax % slew rate
%     diff(delta.physical)./dt.physical(1:end-1) >= -deltaRateMax
    diff(delta.physical)./dt.physical <= deltaRateMax % slew rate
    diff(delta.physical)./dt.physical >= -deltaRateMax
    ];

%% Physical Constraints
% Slip angles
if p.integration == 0
    constraints = [ constraints
        
    tan(alphaF.physical + delta.physical).*Ux.physical(1:N_var)/Uy.const ...
    == (Uy.physical(1:N_var) + a*r.physical(1:N_var))/Uy.const
    
    tan(alphaR.physical).*Ux.physical(1:N_var)/Uy.const                  ...
    == (Uy.physical(1:N_var) - b*r.physical(1:N_var))/Uy.const
    
    ];

elseif p.integration == 1
    constraints = [ constraints
        
    tan(alphaF.physical + Heun(delta.physical)).*Ux.physical(1:N)/Uy.const ...
    == (Uy.physical(1:N) + a*r.physical(1:N))/Uy.const
    
    tan(alphaR.physical).*Ux.physical(1:N)/Uy.const                  ...
    == (Uy.physical(1:N) - b*r.physical(1:N))/Uy.const
    
    tan(alphaF2.physical + Heun(delta.physical)).*Ux.physical(2:N+1)/Uy.const ...
    == (Uy.physical(2:N+1) + a*r.physical(2:N+1))/Uy.const
    
    tan(alphaR2.physical).*Ux.physical(2:N+1)/Uy.const                  ...
    == (Uy.physical(2:N+1) - b*r.physical(2:N+1))/Uy.const
    
    ];
end

if p.integration == 0
% Forces
constraints = [ constraints
    % Longitudinal Forces:
    Fxf.variable == 0 %+ slack.variable % No front brakes for now
    Fxr.variable == Tr.physical/Rwr/Fxr.const %+ slack.variable interesting
    % Tire friction Limit:
    Fxr.variable <=  mur*Fzr.physical/Fxr.const %+ slack.variable
    Fxr.variable >= -mur*Fzr.physical.*cos(alphaR.physical)/Fxr.const %+ slack.variable
    % if input is FxR instead of Tr
    % Fxr.variable        >= Tmin/Rwr/Fxr.const
    % Fxr.variable        <= Tmax/Rwr/Fxr.const
    % Normal loading forces:
    Fzf.variable == (m*b*g - h*Fxr.physical)/(a+b)/Fzf.const %+ slack.variable
    Fzr.variable == (m*a*g + h*Fxr.physical)/(a+b)/Fzr.const %+ slack.variable huge violations. big slack
    % Lateral Forces:
    Fyf.variable.*(1+exp(Wf*alphaF.physical))    ...
                 == muf*Fzf.physical.*(1 - exp(Wf*alphaF.physical))/Fyf.const %+ slack.variable
    
    (Xi.variable*mur.*Fzr.physical/Fzr.const).^2 ...
                 == (mur*Fzr.physical/Fzr.const).^2 - (Fxr.physical/Fzr.const).^2 %+ slack.variable
    %     Xi.variable == ones(size(Xi.variable)) % debug
    Fyr.variable.*(1+exp(Wr*alphaR.physical))    ...
                 == mur*Fzr.physical.*Xi.physical.*(1 - exp(Wr*alphaR.physical))/Fyr.const %+ slack.variable
    
    % (Fxr.physical.^2 + Fyr.physical.^2).*(1 + 2*exp(Wr*alphaR.physical) + exp(2*Wr*alphaR.physical))/Fyr.const^2 == ...
    % ((muR*Fzr.physical).^2.*(1 - 2*exp(Wr*alphaR.physical) + exp(2*Wr*alphaR.physical))/Fyr.const^2;
    ];
elseif p.integration == 1
    constraints = [ constraints
        % Longitudinal Forces:
        Fxf.variable        == 0 % No front brakes for now
        Fxr.variable        == Heun(Tr.physical)/Rwr/Fxr.const 
        % Tire friction Limit:
        Fxr.variable        <=  mur*Fzr.physical/Fxr.const 
        Fxr.variable        >= -mur*Fzr.physical.*cos(alphaR.physical)/Fxr.const 
        % Normal loading forces:
        Fzf.variable        == (m*b*g - h*Fxr.physical)/(a+b)/Fzf.const 
        Fzr.variable        == (m*a*g + h*Fxr.physical)/(a+b)/Fzr.const
        % Lateral Forces:
        Fyf.variable.*(1+exp(Wf*alphaF.physical))    ...
                            == muf*Fzf.physical.*(1 - exp(Wf*alphaF.physical))/Fyf.const        
        (Xi.variable*mur.*Fzr.physical/Fzr.const).^2 ...
                            == (mur*Fzr.physical/Fzr.const).^2 - (Fxr.physical/Fzr.const).^2
        Fyr.variable.*(1+exp(Wr*alphaR.physical))    ...
                            == mur*Fzr.physical.*Xi.physical.*(1 - exp(Wr*alphaR.physical))/Fyr.const
        % Longitudinal Forces:
        Fxf2.variable       == 0 % No front brakes for now
        Fxr2.variable       == Heun(Tr.physical)/Rwr/Fxr2.const 
        % Tire friction Limit:
        Fxr2.variable       <=  mur*Fzr2.physical/Fxr2.const %+ slack.variable
        Fxr2.variable       >= -mur*Fzr2.physical.*cos(alphaR2.physical)/Fxr2.const %+ slack.variable
        % Normal loading forces:
        Fzf2.variable       == (m*b*g - h*Fxr2.physical)/(a+b)/Fzf2.const
        Fzr2.variable       == (m*a*g + h*Fxr2.physical)/(a+b)/Fzr2.const 
        % Lateral Forces:
        Fyf2.variable.*(1+exp(Wf*alphaF2.physical))    ...
                            == muf*Fzf2.physical.*(1 - exp(Wf*alphaF2.physical))/Fyf2.const
        (Xi2.variable*mur.*Fzr2.physical/Fzr2.const).^2 ...
                            == (mur*Fzr2.physical/Fzr2.const).^2 - (Fxr2.physical/Fzr2.const).^2
        Fyr2.variable.*(1+exp(Wr*alphaR2.physical))    ...
                            == mur*Fzr2.physical.*Xi2.physical.*(1 - exp(Wr*alphaR2.physical))/Fyr2.const
        ];
end


%% Objective
objective = ( ...
    + (( xE.physical(N+1-nSS) -   E_f)/xE.const)^2 ... % target East pos
    + (( yN.physical(N+1-nSS) -   N_f)/yN.const)^2 ... % target North pos
...    + ((Psi.physical(N+1-nSS) - Psi_f)/Psi.const)^2 ...% target Psi
    + sum(dt.variable)/N ...                           % minimize time
...    - sum(xE.variable.^2)/N ...                        % maximize lat. dev.
...    + 100*sum(diff(delta.variable).^2)/N ...           % change in delta
    + sum(diff(Tr.variable).^2)/N ...                  % change in Tr
...    + sum(diff(dt.variable).^2)/N ...                  % change in dt
...    + (slack.variable).^2 ...                          % slack
...    + (Ux.variable(N+1) - Ux_f/Ux.const)^2 ...         % drift eq Ux
...    + (Uy.variable(N+1) - Uy_f/Uy.const)^2 ...         % drift eq Uy
...    + (r.variable(N+1)  - r_f/ r.const)^2 ...          % drift eq r
    );
% + sum(delta.variable.^2 + Tr.variable.^2)
% + sum(abs(Tr.variable))/N ...
% + (delta.variable(N) - delta_f/delta.const)^2 ...

%% Collect sdpvars and return
s = whos;
for i = 1:numel(s)
    if strcmp(s(i).class,'mdlvar')
        variables.(s(i).name) = eval(s(i).name);
    end
end

