function xdot = twoWheel_rvb_solverWrapper(params,x,delta,Fxr)
muFzr = params.rtire.mu*params.rtire.Fz;
if abs(Fxr) > abs(muFzr) || abs(delta) > pi/2
   xdot = 1e5*[1 1 1];
else
[xdot, ~] = twoWheel_rvb_v2(params,x,delta,Fxr);
end

end
% twoWheel_rvb_v2(vehicle,[signr*kappa*x(1),x(1),beta],x(2),1000*x(3))