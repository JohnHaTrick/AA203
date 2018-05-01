function [objective, constraints, variables] = DriftNonlinear(p)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

%% Initialize Variables
% INPUT VARIABLES
delta   = mdlvar(N,1,'input');
% Tr      = mdlvar(N,15000,'input');
Fxf     = mdlvar(N,1e4);

% LIFTING VARIABLES
alphaF  = mdlvar(N,1e-3);
alphaR  = mdlvar(N,1e-4);
Fyf     = mdlvar(N,1e4);
% Fxf     = mdlvar(N,1e4);
Fzf     = mdlvar(N,1e4);
Fyr     = mdlvar(N,1e4);
Fxr     = mdlvar(N,1e4);
Fzr     = mdlvar(N,1e4);
% slack   = mdlvar(1,1);

% STATE VARIABLES
xE      = mdlvar(N+1,10,'state');
yN      = mdlvar(N+1,10,'state');
Psi     = mdlvar(N+1,1,'state');
Ux      = mdlvar(N+1,10,'state');
Uy      = mdlvar(N+1,5,'state');
r       = mdlvar(N+1,1,'state');

%% Inizialize Constraints
constraints = [];

% Differential Equations
constraints = [ constraints
    (   diff(xE.variable)         == (- Ux.physical(1:N).*sin(Psi.physical(1:N)) - Uy.physical(1:N).*cos(Psi.physical(1:N)))*dt/xE.const  ): 'East Position'
    (   diff(yN.variable)         == (Ux.physical(1:N).*cos(Psi.physical(1:N)) - Uy.physical(1:N).*sin(Psi.physical(1:N)))*dt/yN.const  ): 'North Position'
    (   diff(Psi.variable)        == r.physical(1:N)*dt/Psi.const  ): 'Orientation'
    
    (   diff(Ux.variable)         == (Fxf.physical.*cos(delta.physical) - Fyf.physical.*sin(delta.physical)...
                                        + Fxr.physical + m*r.physical(1:N).*Uy.physical(1:N))/m*dt/Ux.const  ): 'x Velocity'
    (   diff(Uy.variable)         == (Fxf.physical.*sin(delta.physical) + Fyf.physical.*cos(delta.physical)...
                                        + Fyr.physical - m*r.physical(1:N).*Ux.physical(1:N))/m*dt/Uy.const  ): 'y Velocity'
    (   diff(r.variable)          == (a*Fyf.physical.*cos(delta.physical) + a*Fxf.physical.*sin(delta.physical)...
                                        - b*Fyr.physical)/Iz*dt/r.const ): 'Yaw rate'
    
    (   diff(Ux.variable(end-2:end))         == 0  ): 'x Velocity Eq in the end'
    (   diff(Uy.variable(end-2:end))         == 0  ): 'y Velocity Eq in the end'
    (   diff(r.variable(end-2:end))          == 0  ): 'Yaw rate Eq in the end'
    ];

% State Constraints
constraints = [ constraints
    yN.variable >= 0
    Psi.variable <= pi/Psi.const
    Psi.variable >= -pi/Psi.const
    alphaF.variable <= pi/2/alphaF.const
    alphaF.variable >= -pi/2/alphaF.const
    alphaR.variable <= pi/2/alphaR.const
    alphaR.variable >= -pi/2/alphaR.const
    ];

% Initial Conditions
constraints = [ constraints
    xE.variable(1)      == E_0/xE.const
    yN.variable(1)      == N_0/yN.const
    Psi.variable(1)     == Psi_0/Psi.const
    Ux.variable(1)      == Ux_0/Ux.const
    Uy.variable(1)      == Uy_0/Uy.const
    r.variable(1)       == r_0/r.const
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
    ];

% Input Constraints
constraints = [ constraints
%     Tr.variable        <=  Tmax/Tr.const % Start simpler
%     Tr.variable        >=  Tmin/Tr.const % Start simpler
%     Tr.variable        >= 0
    delta.variable     <=  deltaMax/delta.const
    delta.variable     >= -deltaMax/delta.const
%     delta.variable     == 0
    ];

%% Physical Constraints
% Slip angles
constraints = [ constraints
%     alphaF.variable    == (atan((Uy.physical(1:N) + a*r.physical(1:N))./Ux.physical(1:N)) - delta.physical)/alphaF.const
%     alphaR.variable    == (atan((Uy.physical(1:N) - b*r.physical(1:N))./Ux.physical(1:N)))/alphaR.const
    tan(alphaF.physical + delta.physical).*Ux.physical(1:N)/Uy.const    == (Uy.physical(1:N) + a*r.physical(1:N))/Uy.const
    tan(alphaR.physical).*Ux.physical(1:N)/Uy.const                     == (Uy.physical(1:N) - b*r.physical(1:N))/Uy.const
%     tan(delta.physical).*Ux.physical(1:N)/Uy.const    == (Uy.physical(1:N) + a*r.physical(1:N))/Uy.const % Kinetmatic model
%     (Uy.physical(1:N) - b*r.physical(1:N))/Uy.const   == 0 % Kinetmatic model
    ];

% Forces
constraints = [ constraints
    Fxf.variable        == 0
%     Fxr.variable        == physical(Tr)/Rw/Fxr.const
    Fxr.variable        <= Tmax/Rw/Fxr.const
    Fxr.variable        <= muR*Fzr.physical.*cos(alphaR.physical)/Fxr.const % Tire limit
    Fxr.variable        >= Tmin/Rw/Fxr.const
    Fxr.variable        >= -muR*Fzr.physical.*cos(alphaR.physical)/Fxr.const % Tire limit
    Fzf.variable        == 1/(a+b)*(m*b*g - h*Fxr.physical)/Fzf.const
    Fzr.variable        == 1/(a+b)*(m*a*g + h*Fxr.physical)/Fzr.const

    Fyf.variable.*(1+exp(wF*alphaF.physical))   == muF*Fzf.physical.*(1 - exp(wF*alphaF.physical))/Fyf.const;
    
    Fyr.variable.*(1+exp(wR*alphaR.physical))   == muR*Fzr.physical.*(1 - exp(wR*alphaR.physical))/Fyr.const;
    
    ];

%% Objective
% objective = sum(abs(diff(delta.variable)) + abs(diff(Tr.variable))); % Black magic tricks: set objective size to 1
% objective = sum(diff(delta.variable).^2 + diff(Tr.variable).^2)/1.5e-2; % Black magic tricks: set objective size to 1
objective = ((xE.variable(N+1) - E_f/xE.const)^2 + ...
            (yN.variable(N+1) - N_f/yN.const)^2 + ...
            (Psi.variable(N+1) - Psi_f/Psi.const)^2 + ...
            (Ux.variable(N+1) - Ux_f/Ux.const)^2 + ...
            (Uy.variable(N+1) - Uy_f/Uy.const)^2 + ...
            (r.variable(N+1) - r_f/r.const)^2)/10 + ...
             0.01*sum(diff(delta.variable).^2 + diff(Fxr.variable).^2)/N;
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