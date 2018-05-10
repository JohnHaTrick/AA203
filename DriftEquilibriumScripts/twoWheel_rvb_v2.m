function [xdot,phidot] = twoWheel_rvb_v2(vehicle,x,delta,Fxr)
%% Vehicle Parameters

r = x(1);
v = x(2);
beta = x(3);

m       = vehicle.m;
a       = vehicle.a; % Forward Distance to CG
b       = vehicle.b; % Rearward Distance to CG
Iz     = vehicle.Iz; % Moment of Inertia About Z Axis


% Forces and Inputs
Fxf = 0;

% Slip Angles
vy = v*sin(beta);
vx = v*cos(beta);
alphaF = atan2(vy+a*r,vx) - delta;
alphaR = atan2(vy-b*r,vx);
%alphaF = ((Uy + a*r)/(Ux)) - delta;
%alphaR = ((Uy - b*r)/(Ux));

% Lateral Forces
ftire.Ca = vehicle.Caf;
ftire.mu = vehicle.muf;
ftire.Fz = vehicle.Fzf_stat;
Fyf = fiala2dSimpleCoupling_V2(Fxf,alphaF,ftire);
rtire.Ca = vehicle.Car;
rtire.mu = vehicle.mur;
rtire.Fz = vehicle.Fzr_stat;
Fyr = fiala2dSimpleCoupling_V2(Fxr,alphaR,rtire);

% Equations of Motion
rdot = (1/Iz)*(a*Fyf*cos(delta) - b*Fyr);
phidot = (1/(m*v))*(Fyf*cos(delta-beta)+Fyr*cos(beta)-Fxr*sin(beta));
vdot = (1/m)*(-Fyf*sin(delta-beta)+Fyr*sin(beta)+Fxr*cos(beta));

betadot = phidot-r;

xdot = [rdot, vdot, betadot];
 


end

