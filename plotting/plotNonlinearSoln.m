function plotNonlinearSoln(sol, p)
%PLOTNONLINEARSOLN
%   Plot solution of AA203 nonlinear trajectory

close all;

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


%% State Variables
figure('Name','State Variables','Position',[0 50 800 950])
subplot(5,2,1); hold on; box on
    plot(t,xE,'k'); grid on
    %plot(t,eqStates.E_f*ones(size(t)),'k--')
    ylabel('E [m]')
    set(gca,'xticklabel',{})
subplot(5,2,2); hold on; box on
    plot(t,yN,'k'); grid on
    %plot(t,eqStates.N_f*ones(size(t)),'k--')
    ylabel('N [m]')
    set(gca,'xticklabel',{})
subplot(5,2,3); hold on; box on
    plot(t,Psi/pi*180,'k'); grid on
    %plot(t,eqStates.Psi_f*ones(size(t)),'k--')
    set(gca,'xticklabel',{})
    ylabel('\Psi [deg]')
subplot(5,2,4); hold on; box on;
    plot(t,atan(Uy./Ux)/pi*180,'k'); grid on
    plot(t,p.beta_f*ones(size(t))/pi*180,'k--')
    set(gca,'xticklabel',{})
    ylabel('beta [rad]')
subplot(5,2,5); hold on; box on
    plot(t,r,'k'); grid on
    plot(t,p.r_f*ones(size(t)),'k--')
    ylabel('r [rad/s]')
    set(gca,'xticklabel',{})
subplot(5,2,6); hold on; box on
    plot(t,sqrt(Ux.^2+Uy.^2),'k'); grid on
    plot(t,p.V_f*ones(size(t)),'k--')
    ylabel('V [m/s]')
    set(gca,'xticklabel',{})
    ylim([0 1.1*max(Ux)])
subplot(5,2,7); hold on; box on
    plot(t,Ux,'k'); grid on
    plot(t,p.Ux_f*ones(size(t)),'k--')
    ylabel('U_x [m/s]')
    set(gca,'xticklabel',{})
    ylim([0 1.1*max(Ux)])
subplot(5,2,8); hold on; box on
    plot(t,Uy,'k'); grid on
    plot(t,p.Uy_f*ones(size(t)),'k--')
    xlabel('t [s]')
    ylabel('U_y [m/s]')
    ylim([1.1*min(Uy) 2*max(Uy)])
subplot(5,2,9); hold on; grid on;
    plot(t, .5*(p.Iz*sol.state.r.^2))
    plot(t, .5*(p.m*(Ux.^2+Uy.^2)))
    plot(t, .5*(p.m*(Ux.^2+Uy.^2)+p.Iz*sol.state.r.^2),'--')
    legend('Rotational KE', 'Translational KE', 'Total KE')
    xlabel('t [s]')
    ylabel('Joules')
subplot(5,2,10); hold on; grid on;
    plot(dt*1000)
    title('solution time steps')
    xlabel('iterations')
    ylabel('msec')
    
%% Input Variables
figure('Name','Input Variables','Position',[800 50 400 800])
ax(1) = subplot(311); hold on; box on; grid on
    stairs(t(1:N),Tr(1:N),'k')
    ylabel('T_r [Nm]')
%     ylim([p.Tmin, p.Tmax])
    ylim([-5000, 5000])
    set(gca,'xticklabel',{})
ax(2) = subplot(312); hold on; box on; grid on
    stairs(t(1:N),delta(1:N)*180/pi,'k')
    plot(t,p.delta_f*180/pi*ones(size(t)),'k--')
    plot(t,p.deltaMax*ones(size(t))*180/pi,'--r')
    plot(t,-p.deltaMax*ones(size(t))*180/pi,'--r')
    linkaxes(ax,'x');
    ylabel('\delta [deg]')
    xlabel('t [s]')
    ylim([-1.1*p.deltaMax*180/pi, 1.1*p.deltaMax*180/pi])
ax(3) = subplot(313); hold on; box on; grid on;
    plot(diff(sol.input.delta)./sol.variable.dt(1:end-1),'k')
    plot(p.deltaRateMax*ones(size(diff(sol.input.delta))),'--r')
    plot(-p.deltaRateMax*ones(size(diff(sol.input.delta))),'--r')
    title('Slew Rate')
    xlabel('iteration #')
    ylabel('rad/sec')
    ylim([-2*p.deltaRateMax, 2*p.deltaRateMax])
    
%% Global Position Trajectory
figure('Name','Trajectory E-N','Position',[1200 50 700 700]);
grid on; box on; hold on;
plot(xE,yN,'k--');
axis equal
xlabel('E [m]')
ylabel('N [m]')

% draw MARTY
for j = 0:N
    if mod(j-1,30) == 0
        i = N+1-j;
        w = 1.5;    l = 2;    t = .7;
        R   = plot([xE(i)-w/2, xE(i)+w/2], ...
                   [yN(i)-l/2, yN(i)-l/2],'Color',[.5 .5 .5],'LineWidth',2);
        F   = plot([xE(i)+w/2, xE(i)-w/2], ...
                   [yN(i)+l/2, yN(i)+l/2],'Color',[.5 .5 .5],'LineWidth',2);
        C   = plot([xE(i),     xE(i)], ...
                   [yN(i)-l/2, yN(i)+l/2],'Color',[.5 .5 .5],'LineWidth',2);
        Wrr = plot([xE(i)+w/2, xE(i)+w/2], ...
                   [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',3);
        Wrl = plot([xE(i)-w/2, xE(i)-w/2], ...
                   [yN(i)-l/2-t/2, yN(i)-l/2+t/2],'k','LineWidth',3);
        Wfr = plot([xE(i)+w/2, xE(i)+w/2], ...
                   [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',3);
        Wfl = plot([xE(i)-w/2, xE(i)-w/2], ...
                   [yN(i)+l/2-t/2, yN(i)+l/2+t/2],'k','LineWidth',3);
        rotate(Wfr, [0 0 1], delta(min(i,N))*180/pi, [xE(i)+w/2,yN(i)+l/2,0]);
        rotate(Wfl, [0 0 1], delta(min(i,N))*180/pi, [xE(i)-w/2,yN(i)+l/2,0]);
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

% draw target circle


%% what's this do?
GenerateFig = false;
if GenerateFig
    fileName = 'ICE2018';
    mlf2pdf(gcf,fileName)
    system(strcat('pdfcrop --noverbose ./',fileName,'.pdf ./', fileName,'.pdf'));
end

end

