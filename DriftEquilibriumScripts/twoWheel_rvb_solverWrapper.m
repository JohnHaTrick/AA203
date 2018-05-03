function xdot = twoWheel_rvb_solverWrapper(vehicle,x,delta,Fxr)
muFzr = vehicle.mur*vehicle.Fzr;
if abs(Fxr) > abs(muFzr) || abs(delta) > pi/2
   xdot = 1e5*[1 1 1];
else
[xdot, ~] = twoWheel_rvb_v2(vehicle,x,delta,Fxr);
end

end
% twoWheel_rvb_v2(vehicle,[signr*kappa*x(1),x(1),beta],x(2),1000*x(3))