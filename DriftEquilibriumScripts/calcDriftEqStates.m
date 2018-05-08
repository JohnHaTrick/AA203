function eqStates = calcDriftEqStates(R, beta, vehicle)
%CALCEQDRIFTSTATES Summary of this function goes here
%   Detailed explanation goes here

eqStates.beta = beta;

[eqStates.r,eqStates.V, eqStates.delta,eqStates.Fxr,exitflag] = solveDriftEquilibrium_simple(R,beta,vehicle);

% if exitFlag error catch???

end

