function [objective, constraints, variables] = DriftNonlinear(p,integration)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

if integration == 0 %0-Euler, 1-Heun
    N_var = N;
elseif integration == 1
    N_var = N+1;
    Heun = @(x) (x(1:end-1)+x(2:end))/2;
end

%% Initialize Variables
% INPUT VARIABLES
delta   = mdlvar(N_var,1,'input');
Tr      = mdlvar(N_var,10000,'input');
% Fxr     = mdlvar(N,1e4,'input'); % if using Fxr as input

% LIFTING VARIABLES
alphaF  = mdlvar(N_var,1);
alphaR  = mdlvar(N_var,1);
Fyf     = mdlvar(N_var,1e4);
Fxf     = mdlvar(N_var,1e4);
Fzf     = mdlvar(N_var,1e4);
Fyr     = mdlvar(N_var,1e4);
Fxr     = mdlvar(N_var,1e4);
Fzr     = mdlvar(N_var,1e4);
Xi      = mdlvar(N_var,1);
slack   = mdlvar(N_var,1);
dt      = mdlvar(N_var,1e-2);

% STATE VARIABLES
xE      = mdlvar(N+1,10,'state');
yN      = mdlvar(N+1,10,'state');
Psi     = mdlvar(N+1,1,'state');
Ux      = mdlvar(N+1,10,'state');
Uy      = mdlvar(N+1,4,'state');
r       = mdlvar(N+1,1,'state');


%% Initial Guesses
% INPUT VARIABLES
assign(delta.variable,  zeros(1,N_var))
assign(Tr.variable,     zeros(1,N_var))

% LIFTING VARIABLES
assign(alphaF.variable, zeros(1,N_var))
assign(alphaR.variable, zeros(1,N_var))
assign(Fyf.variable,    zeros(1,N_var))
assign(Fxf.variable,    zeros(1,N_var))
assign(Fzf.variable,    ones(1,N_var))
assign(Fyr.variable,    zeros(1,N_var))
assign(Fxr.variable,    zeros(1,N_var))
assign(Fzr.variable,    ones(1,N_var))
assign(Xi.variable,     ones(1,N_var))
% assign(slack.variable,  zeros(1,N+1))
assign(dt.variable, zeros(1,N_var)+0.01)

% STATE VARIABLES
assign(xE.variable,     zeros(1,N+1))
% assign(yN.variable,     zeros(1,N+1))
assign(Psi.variable,    zeros(1,N+1))
assign(Ux.variable,     ones(1,N+1))
assign(Uy.variable,     zeros(1,N+1))
assign(r.variable,      zeros(1,N+1))


%% Inizialize Constraints
constraints = [];

% Time constraint
constraints = [ constraints
    dt.variable >= dtmin/dt.const
    dt.variable <= dtmax/dt.const
    ];

% Differential Equations, Euler Integration
if integration == 0
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
    
elseif integration == 1
    
    constraints = [ constraints
        ( diff(xE.variable)  == Heun(-   Ux.physical.*sin(Psi.physical) ...
        -    Uy.physical.*cos(Psi.physical).*dt.physical/xE.const )): 'East Position'
    ( diff(yN.variable)  == Heun(    Ux.physical.*cos(Psi.physical) ...
        -    Uy.physical.*sin(Psi.physical)).*dt.physical/yN.const ): 'North Position'
    ( diff(Psi.variable) == Heun(   r.physical).*dt.physical/Psi.const ):                         'Orientation'
    ( diff(Ux.variable)  == Heun(  ( Fxf.physical.*cos(delta.physical) ...
        -   Fyf.physical.*sin(delta.physical) ...
        +   Fxr.physical                      ...
        +   m*r.physical.*Uy.physical)/m).*dt.physical/Ux.const ):     'x Velocity'
    ( diff(Uy.variable)  == Heun(   Fxf.physical.*sin(delta.physical) ...
        +   Fyf.physical.*cos(delta.physical) ...
        +   Fyr.physical                      ...
        -   m*r.physical.*Ux.physical)./m.*dt.physical/Uy.const ):     'y Velocity'
    ( diff(r.variable)   == Heun( a*Fyf.physical.*cos(delta.physical) ...
        + a*Fxf.physical.*sin(delta.physical) ...
        - b*Fyr.physical)/Iz.*dt.physical/r.const ):                            'Yaw rate'
    ];
end
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

% Initial Conditions
constraints = [ constraints
    xE.variable(1)      == E_0/xE.const %+ slack.variable
    yN.variable(1)      == N_0/yN.const %+ slack.variable
    Psi.variable(1)     == Psi_0/Psi.const %+ slack.variable
    %     Ux.variable(1)      == Ux_0/Ux.const
    Uy.variable(1)      == Uy_0/Uy.const %+ slack.variable
    r.variable(1)       == r_0/r.const %+ slack.variable
    ];

% Terminal Conditions
constraints = [ constraints
    %     xE.variable(N+1)    <= E_f/xE.const + slack.physical
    %     xE.variable(N+1)    >= E_f/xE.const - slack.physical
    %     yN.variable(N+1)    <= N_f/yN.const + slack.physical
    %     yN.variable(N+1)    >= N_f/yN.const - slack.physical
    %     Psi.variable(N+1)   <= Psi_f/Psi.const + slack.physical
    %     Psi.variable(N+1)   >= Psi_f/Psi.const - slack.physical
    %     Ux.variable(N+1)    <= Ux_f/Ux.const + slack.physical
    %     Ux.variable(N+1)    >= Ux_f/Ux.const - slack.physical
    %     Uy.variable(N+1)    <= Uy_f/Uy.const + slack.physical
    %     Uy.variable(N+1)    >= Uy_f/Uy.const - slack.physical
    %     r.variable(N+1)     <= r_f/r.const + slack.physical
    %     r.variable(N+1)     >= r_f/r.const - slack.physical
    %     slack.variable      >= 0
    %     xE.variable(N+1)    == E_f/xE.const
    %     yN.variable(N+1)    == N_f/yN.const
    %     Psi.variable(N+1)   == Psi_f/Psi.const
    %     Ux.variable(N+1)    == Ux_f/Ux.const
    %     Uy.variable(N+1)    == Uy_f/Uy.const
    %     r.variable(N+1)     == r_f/r.const
    %     delta.variable(N) == delta_f/delta.const;
    ];

% Input Constraints
constraints = [ constraints
    Tr.variable        <=  Tmax/Tr.const %+ slack.variable
    Tr.variable        >=  Tmin/Tr.const %+ slack.variable
    %     Tr.variable        >= 0 % no brakes
    delta.variable     <=  deltaMax/delta.const %+ slack.variable
    delta.variable     >= -deltaMax/delta.const %+ slack.variable
    %     delta.variable     == 0 % no steering
    ];

%% Physical Constraints
% Slip angles
constraints = [ constraints
    % alphaF.variable == (atan((Uy.physical(1:N) + a*r.physical(1:N))./Ux.physical(1:N)) - delta.physical)/alphaF.const
    % alphaR.variable == (atan((Uy.physical(1:N) - b*r.physical(1:N))./Ux.physical(1:N)))/alphaR.const
    % remove division
    tan(alphaF.physical + delta.physical).*Ux.physical(1:N_var)/Uy.const ...
    == (Uy.physical(1:N_var) + a*r.physical(1:N_var))/Uy.const %+ slack.variable
    tan(alphaR.physical).*Ux.physical(1:N_var)/Uy.const                  ...
    == (Uy.physical(1:N_var) - b*r.physical(1:N_var))/Uy.const %+ slack.variable
    % Kinematic Model:
    % tan(delta.physical).*Ux.physical(1:N)/Uy.const  == (Uy.physical(1:N) + a*r.physical(1:N))/Uy.const
    % (Uy.physical(1:N) - b*r.physical(1:N))/Uy.const == 0
    ];

% Forces
constraints = [ constraints
    % Longitudinal Forces:
    Fxf.variable        == 0 %+ slack.variable % No front brakes for now
    Fxr.variable        == Tr.physical/Rwr/Fxr.const %+ slack.variable interesting
    % Tire friction Limit:
    Fxr.variable        <=  mur*Fzr.physical/Fxr.const %+ slack.variable
    Fxr.variable        >= -mur*Fzr.physical.*cos(alphaR.physical)/Fxr.const %+ slack.variable
    % if input is FxR instead of Tr
    % Fxr.variable        >= Tmin/Rwr/Fxr.const
    % Fxr.variable        <= Tmax/Rwr/Fxr.const
    % Normal loading forces:
    Fzf.variable        == (m*b*g - h*Fxr.physical)/(a+b)/Fzf.const %+ slack.variable
    Fzr.variable        == (m*a*g + h*Fxr.physical)/(a+b)/Fzr.const %+ slack.variable huge violations. big slack
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


%% Objective
% objective = sum(abs(diff(delta.variable)) + abs(diff(Tr.variable))); % Black magic tricks: set objective size to 1
% objective = sum(diff(delta.variable).^2 + diff(Tr.variable).^2)/1.5e-2; % Black magic tricks: set objective size to 1
objective = ( ...
    + (Ux.variable(N+1)  - Ux_f/Ux.const)^2 ...
    + (Uy.variable(N+1)  - Uy_f/Uy.const)^2 ...
    + (r.variable(N+1)   - r_f/r.const)^2 ...
    + (delta.variable(N) - delta_f/delta.const)^2 ...
    + sum(abs(Tr.variable))/N ...
    + sum(diff(delta.variable).^2)/N ...
    + norm(slack.variable) ...
    );
%             + (slack.variable).^2 ...
%             + (xE.variable(N+1)  - E_f/xE.const)^2 ...
%             + (yN.variable(N+1)  - N_f/yN.const)^2 ...
%             + (Psi.variable(N+1) - Psi_f/Psi.const)^2 ...

% objective = slack.variable + 0*sum(delta.variable.^2 + Tr.variable.^2);
% objective = slack.variable ... %+ 0.001*sum(delta.variable.^2 + Fxr.variable.^2)...
%             + 0.01*sum(diff(delta.variable).^2 + diff(Fxr.variable).^2);


%% Collect sdpvars and return
s = whos;
for i = 1:numel(s)
    if strcmp(s(i).class,'mdlvar')
        variables.(s(i).name) = eval(s(i).name);
    end
end