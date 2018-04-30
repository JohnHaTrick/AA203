function [objective, constraints, variables] = DriftNonlinear(p)

% Copy paramteres
copynames = fieldnames(p);
for i = 1:length(copynames)
    eval(strcat(copynames{i},' = p.(copynames{',num2str(i),'});'))
end

%% Initialize Variables
% INPUT VARIABLES
delta   = mdlvar(N,1,'input');
Tr      = mdlvar(N,15000,'input');

% LIFTING VARIABLES
% beta    = mdlvar(N,1); % Useless so far...
alphaF  = mdlvar(N,1e-3);
alphaR  = mdlvar(N,1e-4);
Fyf     = mdlvar(N,1e4);
Fxf     = mdlvar(N,1e4);
Fzf     = mdlvar(N,1e4);
Fyr     = mdlvar(N,1e4);
Fxr     = mdlvar(N,1e4);
Fzr     = mdlvar(N,1e4);
Xi      = mdlvar(N,1);

% STATE VARIABLES
xE      = mdlvar(N+1,10,'state');
yN      = mdlvar(N+1,10,'state');
Psi     = mdlvar(N+1,13,'state');
Ux      = mdlvar(N+1,10,'state');
Uy      = mdlvar(N+1,5,'state');
r       = mdlvar(N+1,1,'state');

%% Inizialize Constraints
constraints = [];

% Differential Equations
constraints = [ constraints
    (   diff(xE.variable)          == (Ux.physical(1:N).*cos(Psi.physical(1:N)) - Uy.physical(1:N).*sin(Psi.physical(1:N)))*dt/xE.const  ): 'East Position'
    (   diff(yN.variable)          == (Ux.physical(1:N).*sin(Psi.physical(1:N)) + Uy.physical(1:N).*cos(Psi.physical(1:N)))*dt/yN.const  ): 'North Position'
    (   diff(Psi.variable)        == r.physical(1:N)*dt/Psi.const  ): 'Orientation'
    
    (   diff(Ux.variable)         == (Fxf.physical.*cos(delta.physical) - Fyf.physical.*sin(delta.physical)...
                                        + Fxr.physical + m*r.physical(1:N).*Uy.physical(1:N))/m*dt/Ux.const  ): 'x Velocity'
    (   diff(Uy.variable)         == (Fxf.physical.*sin(delta.physical) + Fyf.physical.*cos(delta.physical)...
                                        + Fyr.physical  - m*r.physical(1:N).*Ux.physical(1:N))/m*dt/Uy.const  ): 'x Velocity'
    (   diff(r.variable)          == (a*Fyf.physical.*cos(delta.physical) + a*Fxf.physical.*sin(delta.physical)...
                                        - b*Fyr.physical)/Iz*dt/r.const ): 'Yaw rate'
    ];

% State Constraints
constraints = [ constraints
    yN.variable >= 0
    Psi.variable <= pi/Psi.const
    Psi.variable >= -pi/Psi.const
    alphaF.variable <= pi/6/alphaF.const
    alphaF.variable >= -pi/6/alphaF.const
    alphaR.variable <= pi/6/alphaR.const
    alphaR.variable >= -pi/6/alphaR.const
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
    xE.variable(N+1)    == E_f/xE.const
    yN.variable(N+1)    == N_f/yN.const
    Psi.variable(N+1)   == Psi_f/Psi.const
    Ux.variable(N+1)    == Ux_f/Ux.const
    Uy.variable(N+1)    == Uy_f/Uy.const
    r.variable(N+1)     == r_f/r.const
    ];

% Input Constraints
constraints = [ constraints
    Tr.variable        <=  Tmax/Tr.const % Start simpler
    Tr.variable        >=  Tmin/Tr.const % Start simpler
    delta.variable     <=  deltaMax/delta.const
    delta.variable     >= -deltaMax/delta.const
    ];

%% Physical Constraints
% Slip angles
constraints = [ constraints
    alphaF.variable    == (atan((Uy.physical(1:N) + a*r.physical(1:N))./Ux.physical(1:N)) - delta.physical)/alphaF.const
    alphaR.variable    == (atan((Uy.physical(1:N) - b*r.physical(1:N))./Ux.physical(1:N)))/alphaF.const
    ];

% Forces
constraints = [ constraints
    Fxf.variable       == 0
    Fxr.variable       == physical(Tr)/Rw/Fxr.const
    Fzf.variable       == 1/(a+b)*(m*b*g - h*Fxr.physical)/Fzf.const
    Fzf.variable       == 1/(a+b)*(m*a*g + h*Fxr.physical)/Fzr.const
    
    Fyf.variable       == -CaF*tan(alphaF.physical) + CaF^2./(3*muF*physical(Fzf)).*abs(tan(physical(alphaF))).*tan(physical(alphaF))...
                            - CaF^3./(27*muF^2*physical(Fzf).^2).*tan(physical(alphaF)).^3
                            
    Fyr.variable       == -CaR*tan(alphaR.physical) + Xi.physical.*CaR^2./(3*muR*physical(Fzr)).*abs(tan(physical(alphaR))).*tan(physical(alphaR))...
                            - CaF^3./(27*muR^2*physical(Fzr).^2).*tan(physical(alphaR)).^3
    
%     Fyf.variable       == 0
                            
%     Fyr.variable       == 0
                            
    physical(Xi).^2.*((muR*physical(Fzr)).^2-physical(Fxr))/Fzr.const == (muR*physical(Fzr)).^2/Fzr.const % Use inverted Xi, such that don't divide by it
    ];

%% Objective
% objective = sum(abs(diff(delta.variable)) + abs(diff(Tr.variable))); % Black magic tricks: set objective size to 1
objective = 0; %sum(diff(delta.variable).^2 + diff(Tr.variable).^2)/1.5e-2; % Black magic tricks: set objective size to 1


%% Collect sdpvars and return
s = whos;
for i = 1:numel(s)
	if strcmp(s(i).class,'mdlvar')
		variables.(s(i).name) = eval(s(i).name);
	end
end