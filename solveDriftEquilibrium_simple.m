function [rSol,vSol, deltaSol,FxrSol,exitflag] = solveDriftEquilibrium_simple(R0,beta,vehicle)

lmset = {'levenberg-marquardt',0.01};%default lambda = 0.01
options = optimset('TolFun', 1e-6,'Display','off','TolX',1e-9,'TypicalX',[.1 .1 1]');
signr = -sign(beta);
guessFxr = 0.25*vehicle.rtire.mu*vehicle.rtire.Fz;
kappa = 1/R0;

guess = [1 0 (1e-3)*guessFxr];
[x_eq, fval, exitflag, ~] = fsolve(@(x) twoWheel_rvb_solverWrapper(vehicle,[signr*kappa*x(1),x(1),beta],x(2),1000*x(3)),guess,options);

rSol = signr*kappa*x_eq(1);
vSol = x_eq(1);
deltaSol = x_eq(2);
FxrSol = 1000*x_eq(3);

end
