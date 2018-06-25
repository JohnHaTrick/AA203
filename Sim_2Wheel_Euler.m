clear all

%load data
load AA203_result_variables.mat

%%
%time variables
t.step = 1e-4; %1ms
t.set = sol.t(1):t.step:sol.t(end);
t.interval = diff(floor(sol.t./t.step));

%initialize state variables
s.xE = zeros(numel(t.set),1);
s.xE = zeros(numel(t.set),1);
s.yN = zeros(numel(t.set),1);
s.Psi = zeros(numel(t.set),1);
s.Ux = zeros(numel(t.set),1);
s.Uy = zeros(numel(t.set),1);
s.r = zeros(numel(t.set),1);

%initialize lifting variables
alphaF = zeros(numel(t.set),1);
alphaR = zeros(numel(t.set),1);
Fxf = zeros(numel(t.set),1);
Fxr = zeros(numel(t.set),1);
Fyf = zeros(numel(t.set),1);
Fyr = zeros(numel(t.set),1);
Fzf = zeros(numel(t.set),1);
Fzr = zeros(numel(t.set),1);
Xi = zeros(numel(t.set),1);

%initialize input variables
in.delta = zeros(numel(t.set)-1,1);
in.Tr = zeros(numel(t.set)-1,1);

%initialize iteration
k = 1;

%initial conditions
s.xE(k) = p.E_0;
s.yN(k) = p.N_0;
s.Psi(k) = p.Psi_0;
s.Ux(k) = p.Ux_0;
s.Uy(k) = p.Uy_0;
s.r(k) = p.r_0;

alphaR(k) = sol.variable.alphaR(k);
alphaF(k) = sol.variable.alphaF(k);
Fxr(k) = sol.variable.Fxr(k);
Fyf(k) = sol.variable.Fyf(k);
Fyr(k) = sol.variable.Fyr(k);
Fzf(k) = sol.variable.Fzf(k);
Fzr(k) = sol.variable.Fzr(k);
Xi(k) = sol.variable.Xi(k);

for i = 1:numel(sol.t)-1 %for each control time step (~9ms)
    for j = 1:t.interval(i) %for each sim time step (1ms)
        k = k+1;
        
        in.delta(k-1) = sol.input.delta(i);
        in.Tr(k-1) = sol.input.Tr(i);
        
        s.xE(k) = s.xE(k-1)+(-s.Ux(k-1)*sin(s.Psi(k-1))-s.Uy(k-1)*cos(s.Psi(k-1)))*t.step;
        s.yN(k) = s.yN(k-1)+(s.Ux(k-1)*cos(s.Psi(k-1))-s.Uy(k-1)*sin(s.Psi(k-1)))*t.step;
        s.Psi(k) = s.Psi(k-1)+s.r(k-1)*t.step;
        s.Ux(k) = s.Ux(k-1)+(Fxf(k-1)*cos(sol.input.delta(i))-Fyf(k-1)*sin(sol.input.delta(i))+Fxr(k-1)+p.m*s.r(k-1)*s.Uy(k-1))/p.m*t.step;
        s.Uy(k) = s.Uy(k-1)+(Fxf(k-1)*sin(sol.input.delta(i))+Fyf(k-1)*cos(sol.input.delta(i))+Fyr(k-1)-p.m*s.r(k-1)*s.Ux(k-1))/p.m*t.step;
        s.r(k) = s.r(k-1)+(p.a*Fyf(k-1)*cos(sol.input.delta(i))+p.a*Fxf(k-1)*sin(sol.input.delta(i))-p.b*Fyr(k-1))/p.Iz*t.step;
        
        alphaR(k) = atan((s.Uy(k-1)-p.b*s.r(k-1))/s.Ux(k-1));
        alphaF(k) = atan((s.Uy(k-1)+p.a*s.r(k-1))/s.Ux(k-1)) - sol.input.delta(i);
        Fxr(k) = sol.input.Tr(i)/p.Rwr;
        Fzf(k) = (p.m*p.b*p.g-p.h*Fxr(k-1))/(p.a+p.b);
        Fzr(k) = (p.m*p.a*p.g+p.h*Fxr(k-1))/(p.a+p.b);
        Xi(k) = sqrt((p.mur*Fzr(k-1))^2-Fxr(k-1)^2)/p.mur/Fzr(k-1);
        Fyf(k) = p.muf*Fzf(k-1)*(1-exp(p.Wf*alphaF(k-1)))/(1+exp(p.Wf*alphaF(k-1)));
        Fyr(k) = p.mur*Fzr(k-1)*Xi(k-1)*(1-exp(p.Wr*alphaR(k-1)))/(1+exp(p.Wr*alphaR(k-1)));
    end
end
close all; 
sim.state = s;
sim.variable = struct('alphaR',alphaR,'alphaF',alphaF,'Fxf',Fxf,'Fxr',Fxr,'Fyf',Fyf,'Fyr',Fyr,'Fzf',Fzf,...
    'Fzr',Fzr,'Xi',Xi,'dt',t.step);
sim.input = in;
sim.t = t.set;
sim.t_interval = t.interval;
p_sim = p;
p_sim.N = sum(t.interval(1:p.N));
%%
subplot(1,2,2);
plotNonlinearSoln_sim(sim,p_sim);
title('Simulation, Euler Forward, dt = 1e-4s')
%%
subplot(1,2,1);
plotNonlinearSoln_sol(sol,p);
title('Original Solution')