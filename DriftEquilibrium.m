function [objective, constraints, variables] = DriftEquilibrium(p)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

%% Initialize Variables
% INPUT VARIABLES
delta   = mdlvar(1,1,'input');
Tr      = mdlvar(1,10000,'input');
% Fxr     = mdlvar(N,1e4,'input'); % if using Fxr as input

% LIFTING VARIABLES
alphaF  = mdlvar(1,1);
alphaR  = mdlvar(1,1);
Fyf     = mdlvar(1,1e4);
Fxf     = mdlvar(1,1e4);
Fzf     = mdlvar(1,1e4);
Fyr     = mdlvar(1,1e4);
Fxr     = mdlvar(1,1e4);
Fzr     = mdlvar(1,1e4);
Xi      = mdlvar(1,1);

% STATE VARIABLES
Ux      = mdlvar(1,10,'state');
Uy      = mdlvar(1,4,'state');
r       = mdlvar(1,1,'state');


%% Initial Guesses
% INPUT VARIABLES
% assign(delta.variable,  zeros(1,N)) 
% assign(Tr.variable,     zeros(1,N))

% LIFTING VARIABLES
% assign(alphaF.variable, zeros(1,N))
% assign(alphaR.variable, zeros(1,N))
% assign(Fyf.variable,    zeros(1,N))
% assign(Fxf.variable,    zeros(1,N))
% assign(Fzf.variable,    ones(1,N))
% assign(Fyr.variable,    zeros(1,N))
% assign(Fxr.variable,    zeros(1,N))
% assign(Fzr.variable,    ones(1,N))
% assign(Xi.variable,     ones(1,N))
% assign(slack.variable,  zeros(1,N+1))

% STATE VARIABLES
% assign(xE.variable,     zeros(1,N+1))
% % assign(yN.variable,     zeros(1,N+1))
% assign(Psi.variable,    zeros(1,N+1))
% assign(Ux.variable,     ones(1,N+1))  
% assign(Uy.variable,     zeros(1,N+1))  
% assign(r.variable,      zeros(1,N+1))


%% Inizialize Constraints
constraints = [];

% Differential Equations, Euler Integration
constraints = [ constraints
    ( 0  == (   Fxf.physical.*cos(delta.physical) ...
                            -   Fyf.physical.*sin(delta.physical) ...
                            +   Fxr.physical                      ...
                            +   m*r.physical.*Uy.physical)/m*dt/Ux.const ):     'x Velocity'
    ( 0  == (   Fxf.physical.*sin(delta.physical) ...
                            +   Fyf.physical.*cos(delta.physical) ...
                            +   Fyr.physical                      ...
                            -   m*r.physical.*Ux.physical)/m*dt/Uy.const ):     'y Velocity'
    ( 0   == ( a*Fyf.physical.*cos(delta.physical) ...
                            + a*Fxf.physical.*sin(delta.physical) ...
                            - b*Fyr.physical)/Iz*dt/r.const ):                            'Yaw rate'
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

% Input Constraints
constraints = [ constraints
    Tr.variable        <=  Tmax/Tr.const %+ slack.variable
    Tr.variable        >=  Tmin/Tr.const %+ slack.variable
%     Tr.variable        >= 0 % no brakes
    delta.variable     <= 0*deltaMax/delta.const %+ slack.variable
    delta.variable     >= -deltaMax/delta.const %+ slack.variable
    delta.variable     == delta_f/delta.const % no steering
    ];

%% Physical Constraints
% Slip angles
constraints = [ constraints
    % alphaF.variable == (atan((Uy.physical(1:N) + a*r.physical(1:N))./Ux.physical(1:N)) - delta.physical)/alphaF.const
    % alphaR.variable == (atan((Uy.physical(1:N) - b*r.physical(1:N))./Ux.physical(1:N)))/alphaR.const
% remove division
    tan(alphaF.physical + delta.physical).*Ux.physical/Uy.const ... 
        == (Uy.physical + a*r.physical)/Uy.const %+ slack.variable
    tan(alphaR.physical).*Ux.physical/Uy.const                  ...
        == (Uy.physical - b*r.physical)/Uy.const %+ slack.variable
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
objective = ( ...
            + (Ux.variable  - Ux_f/Ux.const)^2 ...
            + (Uy.variable  - Uy_f/Uy.const)^2 ...
            + (r.variable   - r_f/r.const)^2 ...
            + (delta.variable - delta_f/delta.const)^2 ...
            );


%% Collect sdpvars and return
s = whos;
for i = 1:numel(s)
	if strcmp(s(i).class,'mdlvar')
		variables.(s(i).name) = eval(s(i).name);
	end
end