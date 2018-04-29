R0 = 5; %m
beta = -0.610865; %-35 deg (same as phi)
vehicle = setup_dmc;
[rSol,vSol, deltaSol,FxrSol,exitflag] = solveDriftEquilibrium_simple(R0,beta,vehicle);