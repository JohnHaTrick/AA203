function interpResults = interpolateResults( sol, p )
%% INTERPOLATERESULTS
%   Create a struct of result variables with a constant time step size
%   for the purpose of making a video

% interps needed: (look to makeDriftMovie.m)
%   time:   [ t, dt ]
%   inputs: [ Tr, delta ]
%   states: [ xE, yN, Psi, Ux, Uy, r ]

% make time arrays
t0 = sol.t(1);
tf = sol.t(end);
N  = (tf-t0)/p.dtmin + 1;
interpResults.t     = linspace(t0,tf,N);
interpResults.dt    = diff(interpResults.t);

% make input array
interpResults.Tr    = interp1(sol.t(1:end-1),sol.input.Tr,interpResults.t);
interpResults.delta = interp1(sol.t(1:end-1),sol.input.delta,interpResults.t);

% make state arrays
interpResults.xE    = interp1(sol.t,sol.state.xE,interpResults.t);
interpResults.yN    = interp1(sol.t,sol.state.yN,interpResults.t);
interpResults.Psi   = interp1(sol.t,sol.state.Psi,interpResults.t);
interpResults.Ux    = interp1(sol.t,sol.state.Ux,interpResults.t);
interpResults.Uy    = interp1(sol.t,sol.state.Uy,interpResults.t);
interpResults.r     = interp1(sol.t,sol.state.r,interpResults.t);

end

