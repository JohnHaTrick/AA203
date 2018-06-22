close all;
clear;

vehicle_X1;

%% tire modelling first
% parameters
% slip angle
alphaMax = 20*pi/180;
alpha = linspace(-alphaMax,0,1000);
% Fiala params
mu       = vehicle.mu_slide;
mu_p     = vehicle.mu_peak;
Fz       = 9.81*950;
Ca       = vehicle.Caf*1.2;

% calculate Fiala anbd Logit Forces
Fy_fiala   = nan(length(alpha),length(mu)*length(Ca));
Fy_fiala_p = nan(length(alpha),length(mu)*length(Ca));
Fy_logit   = nan(length(alpha),length(mu)*length(Ca));
% step through each combination of mu and Ca
c = 0;
for a = 1:length(mu)
    for b = 1:length(Ca)
        c = c+1;
        
        % fiala
        for i = 1:length(alpha)
            if abs(alpha(i)) > atan2(3*mu(a)*Fz,Ca(b))
                Fy_fiala(i,c) = -sign(alpha(i))*mu(a)*Fz;
            else
                Fy_fiala(i,c) = -Ca(b)*tan(alpha(i)) ...
                               + Ca(b)^2*abs(tan(alpha(i)))*tan(alpha(i))/3/mu(a)/Fz ...
                               - Ca(b)^3*tan(alpha(i))^3/27/mu(a)^2/Fz^2;
            end
        end
        
        % fiala_p
        for i = 1:length(alpha)
            if abs(alpha(i)) > atan2(3*mu_p(a)*Fz,Ca(b))
                Fy_fiala_p(i,c) = -sign(alpha(i))*mu(a)*Fz;
            else
                Fy_fiala_p(i,c) = -Ca(b)*tan(alpha(i)) ...
                               + Ca(b)^2*(2-mu(a)/mu_p(a))*abs(tan(alpha(i)))*tan(alpha(i))/3/mu_p(a)/Fz ...
                               - Ca(b)^3*(1-2*mu(a)/3/mu_p(a))*tan(alpha(i))^3/9/mu_p(a)^2/Fz^2;
            end
        end
        
        % logit
        w = 2*Ca(b)/mu(a)/Fz*.92;
        Fy_logit(:,c) = mu(a)*Fz*(1 - 2*exp(w*alpha)./(1 + exp(w*alpha)));
        
    end
end


%% Next, X1 data
load('brandon_rampsteer_2018-04-09_ac.mat');

% Get experiment time

expStart = find(Driver.switchAutosteer_bool==1, 1, 'first');
expEnd   = find(Driver.brakePedal_bool==1, 1, 'first');
exp_time = expStart:ceil(expEnd*(4/5));
dt       = .002; % default MicroAutobox dt
% exp time is 12.97 to 33.3 sec

% % Longitudinal motion signals
% f1 = figure(1);
% set(f1, 'Units', 'normalized', 'Position', [.1 .1 .8 .8]);
% subplot(2,1,1);
%     yyaxis left;
%     plot(t, Driver.switchAutosteer_bool);
%     yyaxis right;
%     hold on;
%     plot(t, Motor.TorqueCmd_Nm);
%     plot(t, Motor.TorqueCmdSent_Nm);
%     plot(t, Motor.TorqueFb_Nm);
%     legend('autoSteer','TorqueCmd','TorqueCmdSent','TorqueFb');
% subplot(2,1,2);
%     yyaxis left;
%     plot(t, GPSINS.Vx_mps);
%     yyaxis right;
%     plot(t, Driver.switchCruise_bool);
%     legend('Vx','Cruise');

% % Global Position
% f1 = figure(1); hold on;
% set(f1, 'Units', 'normalized', 'Position', [.1 .1 .8 .8]);
% % plot(GPSINS.posE_m,GPSINS.posN_m,'--')
% plot(GPSINS.posE_m(exp_time),GPSINS.posN_m(exp_time))
% xlabel('East')
% ylabel('North')

% % Lateral motion signals
% f2 = figure(2);
% set(f2, 'Units', 'normalized', 'Position', [.1 .1 .8 .8]);
% hold on;
% yyaxis right;
% % plot(t, GPSINS.Vy_mps,'--');
% plot(t(exp_time), GPSINS.Vy_mps(exp_time));
% ylabel('Vy')
% yyaxis left;
% % plot(t, GPSINS.yawRate_radps,'--');
% plot(t(exp_time), GPSINS.yawRate_radps(exp_time));
% xlabel('time [sec]')
% ylabel('r')

% % Tire curves for individual wheels
% f3 = figure(3);
% set(f3, 'Units', 'normalized', 'Position', [.1 .1 .8 .8]);
% 
% % Slip angles
% alphafl = atan((GPSINS.Vy_mps + vehicle.a*GPSINS.yawRate_radps) ./ ...
%     (GPSINS.Vx_mps - vehicle.df/2 * GPSINS.yawRate_radps)) - Steering.steerAngleFL_rad;
% alphafr = atan((GPSINS.Vy_mps + vehicle.a*GPSINS.yawRate_radps) ./ ...
%     (GPSINS.Vx_mps + vehicle.df/2 * GPSINS.yawRate_radps)) - Steering.steerAngleFR_rad;
% alpharl = atan((GPSINS.Vy_mps - vehicle.b*GPSINS.yawRate_radps) ./ ...
%     (GPSINS.Vx_mps - vehicle.dr/2 * GPSINS.yawRate_radps)); % - Steering.steerAngleRL_rad;
% alpharr = atan((GPSINS.Vy_mps - vehicle.b*GPSINS.yawRate_radps) ./ ...
%     (GPSINS.Vx_mps + vehicle.dr/2 * GPSINS.yawRate_radps)); % - Steering.steerAngleRR_rad;

ax = GPSINS.accelX_mps2; % - GPSINS.yawRate_radps .* GPSINS.Vy_mps;
ay = GPSINS.accelY_mps2; % + GPSINS.yawRate_radps .* GPSINS.Vx_mps;
 
% Solving for Fxr, Fyf, Fyr based on bicycle model equations of motion
F = zeros(3,length(t));

for i = 1:length(t)-1
    A = [1, sin(Steering.steerAngleFComposite_rad(i)), 0;...
         0, cos(Steering.steerAngleFComposite_rad(i)), 1;...
         0, vehicle.a * cos(Steering.steerAngleFComposite_rad(i)), -vehicle.b];
    B = [vehicle.m * ax(i);...
         vehicle.m * ay(i);...
         vehicle.Izz * (GPSINS.yawRate_radps(i+1)-GPSINS.yawRate_radps(i))/dt];
    F(:,i) = linsolve(A,B);
end

Fxr = F(1,:);
Fyf = F(2,:);
Fyr = F(3,:);

windowSize = 10; 
b   = (1/windowSize)*ones(1,windowSize);
a   = 1;
Fyf = filter(b,a,Fyf);

% 
% % Lateral roll weight transfer effects
% 
% dFzf_lat = 1/vehicle.df * (vehicle.kphif * GPSINS.rollAngle_rad ...
%     + vehicle.hf * (vehicle.mfl + vehicle.mfr) * ay);
% dFzr_lat = 1/vehicle.dr * (vehicle.kphir * GPSINS.rollAngle_rad ...
%     + vehicle.hr * (vehicle.mrl + vehicle.mrr) * ay);
% 
% Fyfl = Fyf/2 - dFzf_lat;
% Fyfr = Fyf/2 + dFzf_lat;
% Fyrl = Fyr/2 - dFzr_lat;
% Fyrr = Fyr/2 + dFzr_lat;
% 
% subplot(2,2,1);
% plot(rad2deg(alphafl(exp_time)),Fyfl(exp_time), '.');
% title('front left');
% subplot(2,2,2);
% plot(rad2deg(alphafr(exp_time)),Fyfr(exp_time), '.');
% title('front right');
% subplot(2,2,3);
% plot(rad2deg(alpharl(exp_time)),Fyrl(exp_time), '.');
% title('rear left');
% subplot(2,2,4);
% plot(rad2deg(alpharr(exp_time)),Fyrr(exp_time), '.');
% title('rear right');


%% Finally, plot


alphaf = atan2(GPSINS.Vy_mps + vehicle.a*GPSINS.yawRate_radps,GPSINS.Vx_mps)...
       - Steering.steerAngleFComposite_rad;
alphar = atan2(GPSINS.Vy_mps - vehicle.b*GPSINS.yawRate_radps,GPSINS.Vx_mps);

Fyf_sat = vehicle.mu_slide(1) * (vehicle.mfl+vehicle.mfr) * 9.81 * [1 1];
Fyr_sat = vehicle.mu_slide(1) * (vehicle.mrl+vehicle.mrr) * 9.81 * [1 1];
x1 = [-20, -4];
x2 = [-13, -4];

f4 = figure(4);
    set(f4, 'Units', 'normalized', 'Position', [.1 .1 .35 .35]);
    hold on; axis square;
    
    xlabel('\alpha  [deg]','fontsize',13)
    ylabel('F_y  [kN]','fontsize',12)
%     ylim([0, max(Fyf(exp_time))/1000])
    xlim([-12, .1])
    ylim([0, 10])
    
    plot( rad2deg(alphaf(exp_time)) + (.6)*ones(size(alphaf(exp_time))), Fyf(exp_time)/1000, '.');
    plot(rad2deg(alpha), Fy_logit(:,1)/1000, 'LineWidth', 5)
    plot(rad2deg(alpha), Fy_fiala_p(:,1)/1000,':','LineWidth', 5)
    lgd = legend('exp. data','logit','Fiala');
    lgd.FontSize = 11;
    
% subplot(2,1,2);
%     hold on;
%     plot(rad2deg(alphar(exp_time)), Fyr(exp_time), '.');
% %     plot(x2, Fyr_sat,'r-');
%     xlabel('degrees')
%     ylabel('Newtons')
    
% figure(5);
% yyaxis left;
% hold on;
% plot(t(exp_time), Fyf(exp_time))
% plot(t(exp_time), Fyr(exp_time))
% plot(t(exp_time), Steering.steerAngleFComposite_rad(exp_time)* 100000, 'k-');
% yyaxis right
% plot(t(exp_time), alphaf(exp_time))
% hold on;
% plot(t(exp_time), alphar(exp_time))

