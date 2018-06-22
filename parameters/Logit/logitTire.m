%% Demonstrate how well A Logit model compares to Fiala

% parameters
% slip angle
alphaMax = 6*pi/180;
alpha = linspace(-alphaMax,alphaMax,1000);
% Fiala params
mu       = [.2,     1];
mu_p     = 1.4*mu;
Fz       = 1000;
Ca       = [60000, 200000];
% alphaLim = atan2(3*mu*Fz,Ca);
% logit params
% w = 2*Ca/mu/Fz;

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
        Fy_logit(:,c) = mu(a)*Fz*(1 - 2*exp(2*Ca(b)/mu(a)/Fz*alpha)./(1 + exp(2*Ca(b)/mu(a)/Fz*alpha)));
        
    end
end


% plot results
figure();
subplot(2,2,1); hold on;
    title('mu = .2, Ca = 60,000')
    plot(alpha*180/pi, Fy_logit(:,1)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,1)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,1)/1000,':','LineWidth',2)
    xlim([-1, 1])
    ylim( 1.1*[ min(Fy_fiala(:,1)), max(Fy_fiala(:,1)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
    legend('logit','fiala(2)','fiala(3)')
subplot(2,2,2); hold on;
    title('mu = .2, Ca = 200,000')
    plot(alpha*180/pi, Fy_logit(:,2)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,2)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,2)/1000,':','LineWidth',2)
    xlim([-.31, .31])
    ylim( 1.1*[ min(Fy_fiala(:,2)), max(Fy_fiala(:,2)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
subplot(2,2,3); hold on;
    title('mu = 1, Ca = 60,000')
    plot(alpha*180/pi, Fy_logit(:,3)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,3)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,3)/1000,':','LineWidth',2)
    ylim( 1.1*[ min(Fy_fiala(:,3)), max(Fy_fiala(:,3)) ]/1000 )
    xlim([-5, 5])
    ylabel('kN')
    xlabel('degrees')
subplot(2,2,4); hold on;
    title('mu = 1, Ca = 200,000')
    plot(alpha*180/pi, Fy_logit(:,4)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,4)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,4)/1000,':','LineWidth',2)
    xlim([-1.5, 1.5])
    ylim( 1.1*[ min(Fy_fiala(:,4)), max(Fy_fiala(:,4)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
    
% plot results zoom
figure();
subplot(2,2,1); hold on;
    title('mu = .2, Ca = 60,000')
    plot(alpha*180/pi, Fy_logit(:,1)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,1)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,1)/1000,':','LineWidth',2)
    xlim([-1, 0])
    ylim( 1.1*[ 0, max(Fy_fiala(:,1)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
    legend('logit','fiala(2)','fiala(3)')
subplot(2,2,2); hold on;
    title('mu = .2, Ca = 200,000')
    plot(alpha*180/pi, Fy_logit(:,2)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,2)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,2)/1000,':','LineWidth',2)
    xlim([-.31, 0])
    ylim( 1.1*[ 0, max(Fy_fiala(:,2)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
subplot(2,2,3); hold on;
    title('mu = 1, Ca = 60,000')
    plot(alpha*180/pi, Fy_logit(:,3)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,3)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,3)/1000,':','LineWidth',2)
    ylim( 1.1*[ 0, max(Fy_fiala(:,3)) ]/1000 )
    xlim([-5, 0])
    ylabel('kN')
    xlabel('degrees')
subplot(2,2,4); hold on;
    title('mu = 1, Ca = 200,000')
    plot(alpha*180/pi, Fy_logit(:,4)/1000,'LineWidth',3)
    plot(alpha*180/pi, Fy_fiala(:,4)/1000,':','LineWidth',2)
    plot(alpha*180/pi, Fy_fiala_p(:,4)/1000,':','LineWidth',2)
    xlim([-1.5, 0])
    ylim( 1.1*[ 0, max(Fy_fiala(:,4)) ]/1000 )
    ylabel('kN')
    xlabel('degrees')
    
    
    
    
