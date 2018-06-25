function plotNonlinearSoln_sim(sol, p)
%PLOTNONLINEARSOLN
%   Plot solution of AA203 nonlinear trajectory

%close all;

%% Parse Solution Data
t           = sol.t;
dt          = sol.variable.dt;
N = size(t,2)-1;
% State Variables
xE          = sol.state.xE;
yN          = sol.state.yN;
Psi         = sol.state.Psi;
Ux          = sol.state.Ux;
Uy          = sol.state.Uy;
r           = sol.state.r;
% Input Variables
Tr          = sol.input.Tr;
% Tr          = sol.input.Fxr*p.Rwr; % if solving for Fxr
delta       = sol.input.delta;

Fzf = sol.variable.Fzf;
Fzr = sol.variable.Fzr;
Fxf = sol.variable.Fxf;
Fxr = sol.variable.Fxr;
Fyf = sol.variable.Fyf;
Fyr = sol.variable.Fyr;
mu  = p.mu;


%% State Variables
% figure('Name','State Variables','Position',[0 50 800 950])
% subplot(5,2,1); hold on; box on
%     plot(t,xE,'k'); grid on
%     %plot(t,eqStates.E_f*ones(size(t)),'k--')
%     ylabel('E [m]')
%     set(gca,'xticklabel',{})
% subplot(5,2,2); hold on; box on
%     plot(t,yN,'k'); grid on
%     %plot(t,eqStates.N_f*ones(size(t)),'k--')
%     ylabel('N [m]')
%     set(gca,'xticklabel',{})
% subplot(5,2,3); hold on; box on
%     plot(t,Psi/pi*180,'k'); grid on
%     %plot(t,eqStates.Psi_f*ones(size(t)),'k--')
%     set(gca,'xticklabel',{})
%     ylabel('\Psi [deg]')
% subplot(5,2,4); hold on; box on;
%     plot(t,atan(Uy./Ux)/pi*180,'k'); grid on
%     plot(t,p.beta_f*ones(size(t))/pi*180,'k--')
%     set(gca,'xticklabel',{})
%     ylabel('beta [rad]')
% subplot(5,2,5); hold on; box on
%     plot(t,r,'k'); grid on
%     plot(t,p.r_f*ones(size(t)),'k--')
%     ylabel('r [rad/s]')
%     set(gca,'xticklabel',{})
% subplot(5,2,6); hold on; box on
%     plot(t,sqrt(Ux.^2+Uy.^2),'k'); grid on
%     plot(t,p.V_f*ones(size(t)),'k--')
%     ylabel('V [m/s]')
%     set(gca,'xticklabel',{})
%     ylim([0 1.1*max(Ux)])
% subplot(5,2,7); hold on; box on
%     plot(t,Ux,'k'); grid on
%     plot(t,p.Ux_f*ones(size(t)),'k--')
%     ylabel('U_x [m/s]')
%     set(gca,'xticklabel',{})
%     ylim([0 1.1*max(Ux)])
% subplot(5,2,8); hold on; box on
%     plot(t,Uy,'k'); grid on
%     plot(t,p.Uy_f*ones(size(t)),'k--')
%     xlabel('t [s]')
%     ylabel('U_y [m/s]')
%     ylim([1.1*min(Uy) 2*max(Uy)])
% subplot(5,2,9); hold on; grid on;
%     plot(t, .5*(p.Iz*sol.state.r.^2))
%     plot(t, .5*(p.m*(Ux.^2+Uy.^2)))
%     plot(t, .5*(p.m*(Ux.^2+Uy.^2)+p.Iz*sol.state.r.^2),'--')
%     legend('Rotational KE', 'Translational KE', 'Total KE')
%     xlabel('t [s]')
%     ylabel('Joules')
% subplot(5,2,10); hold on; grid on;
%     plot(dt*1000)
%     title('solution time steps')
%     xlabel('stage')
%     ylabel('msec')    
    
%% Presentation states and inputs
% range = 1:length(sol.variable.Fyf);
% figure('Position',[0 50 1200 700])
% subplot(3,3,1); hold on; box on; grid on;
%     title('Global Position')
%     xlabel('East [m]')
%     ylabel('North [m]')
%     xlim([-.5 .5])
%     plot(xE(range),yN(range),'LineWidth',2);
% subplot(3,3,2); hold on; box on; grid on;
%     title('Orientation')
%     xlabel('time [sec]')
%     ylabel('[rad]')
%     plot(t(range),Psi(range)*180/pi,'LineWidth',2);
% subplot(3,3,3); hold on; box on; grid on;
%     title('Steering Angle')
%     xlabel('time [sec]')
%     ylabel('[deg]')
%     stairs(t(range),delta(range)*180/pi,'LineWidth',2)
%     plot(t(range),p.delta_f*180/pi*ones(size(t(range))),'--','LineWidth',2)
% subplot(3,3,4); hold on; box on; grid on;
%     title('Longitudinal Speed')
%     xlabel('time [sec]')
%     ylabel('[m/sec]')
%     plot(t(range),Ux(range),'LineWidth',2)
%     plot(t(range),p.Ux_f*ones(size(t(range))),'--','LineWidth',2)
% subplot(3,3,5); hold on; box on; grid on;
%     title('Lateral Speed')
%     xlabel('time [sec]')
%     ylabel('[m/sec]')
%     plot(t(range),Uy(range),'LineWidth',2)
%     plot(t(range),p.Uy_f*ones(size(t(range))),'--','LineWidth',2)
% subplot(3,3,6); hold on; box on; grid on;
%     title('Rear Axle Torque')
%     xlabel('time [sec]')
%     ylabel('[N-m]')
%     stairs(t(range),Tr(range),'LineWidth',2)
%     plot(t(range),p.Tr_f*ones(size(t(range))),'--','LineWidth',2)
% subplot(3,3,7); hold on; box on; grid on;
%     title('Yaw Rate')
%     xlabel('time [sec]')
%     ylabel('[deg/sec]')
%     plot(t(range),r(range)*180/pi,'LineWidth',2);
%     plot(t(range),p.r_f*ones(size(t(range)))*180/pi,'--','LineWidth',2)
% subplot(3,3,8); hold on; box on; grid on;
%     title('Side Slip')
%     xlabel('time [sec]')
%     ylabel('[deg]')
%     plot(t(range),atan(Uy(range)./Ux(range))/pi*180,'LineWidth',2)
%     plot(t(range),p.beta_f*ones(size(t(range)))/pi*180,'--','LineWidth',2)
% subplot(3,3,9); hold on; box on; grid on;
%     title('Tire Saturation')
%     xlabel('time [sec]')
%     ylabel('[ % ]')
%     plot(t(range), sqrt(Fxf(range).^2+Fyf(range).^2)/mu./Fzf(range),'LineWidth',2)
%     plot(t(range), sqrt(Fxr(range).^2+Fyr(range).^2)/mu./Fzr(range),'LineWidth',2)
%     legend('front axle','rear axle')
    
%% Input Variables
% figure('Name','Input Variables','Position',[800 50 400 800])
% ax(1) = subplot(311); hold on; box on; grid on
%     stairs(t(1:N),Tr(1:N),'k')
%     ylabel('T_r [Nm]')
% %     ylim([p.Tmin, p.Tmax])
%     ylim([-5000, 5000])
%     set(gca,'xticklabel',{})
% ax(2) = subplot(312); hold on; box on; grid on
%     stairs(t(1:N),delta(1:N)*180/pi,'k')
%     plot(t,p.delta_f*180/pi*ones(size(t)),'k--')
%     plot(t,p.deltaMax*ones(size(t))*180/pi,'--r')
%     plot(t,-p.deltaMax*ones(size(t))*180/pi,'--r')
%     linkaxes(ax,'x');
%     ylabel('\delta [deg]')
%     xlabel('t [s]')
%     ylim([-1.1*p.deltaMax*180/pi, 1.1*p.deltaMax*180/pi])
% ax(3) = subplot(313); hold on; box on; grid on;
% %     plot(diff(sol.input.delta)./sol.variable.dt(1:end-1),'k') % use for size(dt) = N
%     plot(diff(sol.input.delta)/sol.variable.dt,'k')
%     plot(p.deltaRateMax*ones(size(diff(sol.input.delta))),'--r')
%     plot(-p.deltaRateMax*ones(size(diff(sol.input.delta))),'--r')
%     title('Slew Rate')
%     xlabel('iteration #')
%     ylabel('rad/sec')
%     ylim([-2*p.deltaRateMax, 2*p.deltaRateMax])
    
%% Global Position Trajectory
% figure('Name','Trajectory E-N','Position',[1200 50 400 700]);
% grid on; box on; hold on; axis equal
% plot(xE,yN,'k--');
% xlabel('E [m]'); ylabel('N [m]')
% ylim([-19,7]); % xlim([]);
% % draw MARTY
% for j = 0:N
% %     if mod(j-1,45) == 0
%     if sum(ismember(j,[0,50,100,150,200,250,295,330,360,385,410,435])) == 1
%         i = N+1-j;
%         w = 1.5;    l = 2;    t = .7;
%         R   = plot([xE(i)-w/2, xE(i)+w/2], ...
%                    [yN(i)-l/2, yN(i)-l/2],'Color',[.5 .5 .5],'LineWidth',2);
%         F   = plot([xE(i)+w/2, xE(i)-w/2], ...
%                    [yN(i)+l/2, yN(i)+l/2],'Color',[.5 .5 .5],'LineWidth',2);
%         C   = plot([xE(i),     xE(i)], ...
%                    [yN(i)-l/2, yN(i)+l/2],'Color',[.5 .5 .5],'LineWidth',2);
%         Wrr = plot([xE(i)+w/2, xE(i)+w/2], ...
%                    [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',3);
%         Wrl = plot([xE(i)-w/2, xE(i)-w/2], ...
%                    [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',3);
%         Wfr = plot([xE(i)+w/2, xE(i)+w/2], ...
%                    [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',3);
%         Wfl = plot([xE(i)-w/2, xE(i)-w/2], ...
%                    [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',3);
%         deltaExtra = delta*2;
%         rotate(Wfr, [0 0 1], deltaExtra(min(i,N))*180/pi, [xE(i)+w/2,yN(i)+l/2,0]);
%         rotate(Wfl, [0 0 1], deltaExtra(min(i,N))*180/pi, [xE(i)-w/2,yN(i)+l/2,0]);
%         rotate(Wrr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(Wrl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(Wfr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(Wfl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(F,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(R,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         rotate(C,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
%         refreshdata
%         drawnow
%     end
% end

%% Global Position Trajectory FOCUS ON DIRFT INITIATION
%figure('Name','Trajectory E-N','Position',[1200 50 400 500]);
grid on; box on; hold on; % axis equal

xlabel('East [m]'); ylabel('North [m]')
% ylim([-15,2]);
% xlim([-4, 4]);
% axis equal
% draw MARTY
carplot = [250,299,340,375,405,433];
for k = 1:numel(carplot)
    carplot(k) = sum(sol.t_interval(1:carplot(k)-1));
end

for j = 0:N
%     if mod(j-1,45) == 0
    if sum(ismember(j,carplot)) == 1
        i = N+1-j;
        w = 1;    l = 2;    t = .7;
        R   = plot([xE(i)-w/2, xE(i)+w/2], ...
                   [yN(i)-l/2, yN(i)-l/2],'Color',[.8 .8 .8],'LineWidth',6);
        F   = plot([xE(i)+w/2, xE(i)-w/2], ...
                   [yN(i)+l/2, yN(i)+l/2],'Color',[.8 .8 .8],'LineWidth',6);
        C   = plot([xE(i),     xE(i)], ...
                   [yN(i)-l/2, yN(i)+l/2],'Color',[.8 .8 .8],'LineWidth',6);
        Wrr = plot([xE(i)+w/2, xE(i)+w/2], ...
                   [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',4);
        Wrl = plot([xE(i)-w/2, xE(i)-w/2], ...
                   [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',4);
        Wfr = plot([xE(i)+w/2, xE(i)+w/2], ...
                   [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',4);
        Wfl = plot([xE(i)-w/2, xE(i)-w/2], ...
                   [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',4);
        deltaExtra = delta*1.5;
        rotate(Wfr, [0 0 1], deltaExtra(min(i,N))*180/pi, [xE(i)+w/2,yN(i)+l/2,0]);
        rotate(Wfl, [0 0 1], deltaExtra(min(i,N))*180/pi, [xE(i)-w/2,yN(i)+l/2,0]);
        rotate(Wrr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(Wrl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(Wfr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(Wfl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(F,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(R,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        rotate(C,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
        refreshdata
        drawnow
    end
end

plot(xE,yN,'r','LineWidth',1.5);

%% Plot Forces
% % rainbow colors
% red   = zeros(size(sol.t(1:length(sol.variable.Fyf))))';
%     red(1:floor(length(red)/3)) = 1;
%     red(floor(length(red)/3):floor(length(red)*2/3)) = linspace(1,0,1+floor(length(red)/3));
% green = ones(size(sol.t(1:length(sol.variable.Fyf))))';
%     green(1:floor(length(red)/3))     = linspace(0,1,floor(length(red)/3));
%     green(floor(length(red)*2/3):end) = linspace(1,0,1+floor(length(red)/3));
% blue  = zeros(size(sol.t(1:length(sol.variable.Fyf))))';
%     blue(floor(length(red)/3):floor(length(red)*2/3)) = linspace(0,1,1+floor(length(red)/3));
%     blue(floor(length(red)*2/3):end) = 1;
% % figure(); hold on;
% %     plot(red,'r')
% %     plot(green,'g')
% %     plot(blue,'b')
% %     legend('red','green','blue')
%     
% sz = 10;
% 
% figure('Name','Force Diagram','Position',[800 400 1000 600]);
% 
% ax(1) = subplot(231); hold on; box on; grid on;
%     title('Steering Angle')
%     ylabel('\delta  [deg]'); xlabel('time  [sec]')
%     ylim([-p.deltaMax*180/pi, p.deltaMax*180/pi])
% %     stairs(t(1:length(sol.variable.Fyf)),delta(1:length(sol.variable.Fyf))*180/pi,'k')
%     for i = 1:length(sol.variable.Fyf)-1
%         stairs(t(i:i+1),delta(i:i+1)*180/pi, ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)   
%     end
% 
% ax(2) = subplot(234); hold on; box on; grid on;
%     title('Rear Axle Torque')
%     ylabel('T_r  [Nm]'); xlabel('time  [sec]')
%     ylim([-5000, 5000])
% %     stairs(t(1:length(sol.variable.Fyf)),Tr(1:length(sol.variable.Fyf)),'k')
%     for i = 1:length(sol.variable.Fyf)-1
%         stairs(t(i:i+1),Tr(i:i+1), ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)   
%     end
%     
% ax(3) = subplot(232); hold on; box on; grid on; axis equal;
%     title('Front Axle Friction Circle')
%     xlabel('Fy  [kN]'); ylabel('Fx  [kN]')
%     xlim([-8 8]);       ylim([-8 8])
%     Fmax = p.mu*p.Fzf_stat;
%     rectangle('Position',[-Fmax, -Fmax, 2*Fmax, 2*Fmax]/1000,'Curvature',1,'EdgeColor','r','LineStyle','--');
%     plot(0,0,'r--')
%     legend('static \muN')
%     set(gca,'Xdir','reverse')
% %     plot(     sol.variable.Fyf.*cos(sol.input.delta(1:length(sol.variable.Fyf)))/1000, ...
% %          (-1)*sol.variable.Fyf.*sin(sol.input.delta(1:length(sol.variable.Fyf)))/1000)
%     for i = 1:length(sol.variable.Fyf)-1
%         plot(sol.variable.Fyf(i:i+1).*cos(sol.input.delta(i:i+1))/1000, ...
%             -sol.variable.Fyf(i:i+1).*sin(sol.input.delta(i:i+1))/1000, ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)
%     end
%    
% ax(4) = subplot(235); hold on; box on; grid on; axis equal;
%     title('Rear Axle Friction Circle')
%     xlabel('Fy  [kN]'); ylabel('Fx  [kN]')
%     xlim([-12 12]);     ylim([-12 12])
%     set(gca,'Xdir','reverse')
%     Fmax = p.mu*p.Fzr_stat;
%     rectangle('Position',[-Fmax, -Fmax, 2*Fmax, 2*Fmax]/1000,'Curvature',1,'EdgeColor','r','LineStyle','--');
% %     plot(sol.variable.Fyr/1000, sol.variable.Fxr/1000)
%     for i = 1:length(sol.variable.Fyf)-1
%         plot(sol.variable.Fyr(i:i+1)/1000, ...
%             sol.variable.Fxr(i:i+1)/1000, ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)
%     end
%     
% ax(5) = subplot(233); hold on; box on; grid on;
%     title('Front Axle Tires Forces')
%     ylabel('[kN]'); xlabel('time  [sec]')
%     plot(sol.t(1:length(sol.variable.Fyf)), p.mu*sol.variable.Fzf/1000,'r--')
% %     plot(sol.t(1:length(sol.variable.Fyf)), sqrt(sol.variable.Fxf.^2 + sol.variable.Fyf.^2)/1000)
%     for i = 1:length(sol.variable.Fyf)-1
%         plot(sol.t(i:i+1), ...
%             sqrt(sol.variable.Fxf(i:i+1).^2 + sol.variable.Fyf(i:i+1).^2)/1000, ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)
%     end
%     legend('dynamic \muN')
%     
% ax(6) = subplot(236); hold on; box on; grid on;
%     title('Rear Axle Tires Forces')
%     ylabel('[kN]'); xlabel('time  [sec]')
% %     plot(sol.t(1:length(sol.variable.Fyf)), sqrt(sol.variable.Fxr.^2 + sol.variable.Fyr.^2)/1000)
%     for i = 1:length(sol.variable.Fyf)-1
%         plot(sol.t(i:i+1), ...
%             sqrt(sol.variable.Fxr(i:i+1).^2 + sol.variable.Fyr(i:i+1).^2)/1000, ...
%             'Color',[red(i), green(i), blue(i)], 'LineWidth', 3)
%     end
%     plot(sol.t(1:length(sol.variable.Fyf)), p.mu*sol.variable.Fzr/1000,'r--')
    

%% what's this do?
GenerateFig = false;
if GenerateFig
    fileName = 'ICE2018';
    mlf2pdf(gcf,fileName)
    system(strcat('pdfcrop --noverbose ./',fileName,'.pdf ./', fileName,'.pdf'));
end

end

