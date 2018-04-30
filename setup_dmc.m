function params = setup_dmc()
%last update: august 2016, thunderhill.

car.m = 1700;
car.a = 1.39;
car.b = 1.008;
car.L = car.a+car.b;
car.Mf = car.b*car.m;
car.Mr = car.a*car.m;
%car.Iz = 1*car.m*car.a*car.b;
%car.Iz = 1.25*car.Iz;
car.Iz = 2300;
car.g = 9.8;
car.Fz = car.m*car.g;
car.h = 0.45;
car.hF = car.h;
car.hR = car.h;
car.tF = 1.6;
car.tR = 1.6;
car.LLTDr = 0.75;
car.LLTDf = 1-car.LLTDr;

car.deltaMax = deg2rad(38.5);
car.deltaMin = -car.deltaMax;
%define front tire

ftire.Ca = 75000;
ftire.Fz = car.Fz*car.b/car.L;
ftire.Cx = 3e5;
ftire.radius = 0.29591;         % 205/45R16


%oct 2016
ftire.mus = 0.95;
ftire.mu = ftire.mus;


%define rear tire
rtire.Ca = 275000; %%N/rad
rtire.Fz = car.Fz*car.a/car.L;
rtire.Cx = 3e5;
rtire.radius = 0.32;

%dr = 0.84;
%dr = 0.835;
%%updated october 2016
rtire.mus = 0.95;
%rtire.mu = 1.05;
rtire.mu = rtire.mus;

%define brake parameters

brakes.MuPad = 0.39;        % Approximate provided by Brembo. varies w/ temp
brakes.MuPad_f = 0.265;     % Average from characterization June 2017 varies w/ temp
brakes.MuPad_r = 0.207;     % Average from characterization June 2017 varies w/ temp
brakes.Rrotor = 0.1238;     % Radius m (average)
brakes.maxPadForce = 30000; % Maximum brake pad force (could be 36k?)

car.K = car.Mf/ftire.Ca - car.Mr/rtire.Ca;
params.rtire = rtire;
params.ftire=ftire;
params.car = car;
params.brakes = brakes;
