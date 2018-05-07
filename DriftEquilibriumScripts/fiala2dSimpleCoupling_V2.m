function [Fy] = fiala2dSimpleCoupling_V2(Fx,alpha,params)
%fiala2d(kappa, alpha, Cx, Ca)
%Returns fiala lateral and longitudinal tire forces. 
%(From equation 2.10 in Rami's thesis.)
%kappa: wheelspin (rad)
%alpha: sideslip (rad)
%Cx: longitudinal stiffness (N/1)
%Ca: lateral stiffness (N/rad)
%mus: kinetic friction coefficient (i.e. slip)
%mu: static friction coefficient
%Fz: Normal force
Ca = params.Ca;
mu = params.mu;
Fz = params.Fz;

muFz = mu*Fz;
%sanitize Fx
Fx = min(Fx,0.9999999*muFz);
Fx = max(Fx,-0.9999999*muFz);


eta = sqrt((muFz).^2-Fx.^2)./(muFz);
Fymax = eta.*muFz;

asl = atan(3*Fymax/Ca);


%Fy = -Fymax*sign(alpha);
%if (abs(alpha)<= asl)
Fy = -Ca.*tan(alpha)+Ca^2./(3.*Fymax).*abs(tan(alpha)).*tan(alpha) - Ca.^3./(27*Fymax.^2).*tan(alpha).^3;
Fy = max(Fy,-Fymax);
Fy = min(Fy,Fymax);
%end



end