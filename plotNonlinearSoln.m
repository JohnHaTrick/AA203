function plotNonlinearSoln( t, sol, p, eqStates)
%PLOTNONLINEARSOLN
%   Plot solution of AA203 nonlinear trajectory

close all;

%% Parse Solution Data
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
    plot(t,eqStates.beta*ones(size(t))/pi*180,'k--')
    set(gca,'xticklabel',{})
    ylabel('beta [rad]')
subplot(5,2,5); hold on; box on
    plot(t,r,'k'); grid on
    plot(t,eqStates.r*ones(size(t)),'k--')
    ylabel('r [rad/s]')
    set(gca,'xticklabel',{})
subplot(5,2,6); hold on; box on
    plot(t,sqrt(Ux.^2+Uy.^2),'k'); grid on
    plot(t,eqStates.V*ones(size(t)),'k--')
    ylabel('V [m/s]')
    set(gca,'xticklabel',{})
subplot(5,2,7); hold on; box on
    plot(t,Ux,'k'); grid on
    plot(t,eqStates.Ux*ones(size(t)),'k--')
    ylabel('U_x [m/s]')
    xlabel('t [s]')
    %set(gca,'xticklabel',{})
subplot(5,2,8); hold on; box on
    plot(t,Uy,'k'); grid on
    plot(t,eqStates.Uy*ones(size(t)),'k--')
    ylabel('U_y [m/s]')
    %set(gca,'xticklabel',{})

    
%% Input Variables
figure('Name','Input Variables','Position',[800 50 400 500])
ax(1) = subplot(211); hold on; box on
    stairs(t(1:N),Tr(1:N),'k'); grid on
    ylabel('T_r [Nm]')
%     ylim([p.Tmin, p.Tmax])
    ylim([-5000, 5000])
    set(gca,'xticklabel',{})
ax(2) = subplot(212); hold on; box on
    stairs(t(1:N),delta(1:N)*180/pi,'k'); grid on
    plot(t,eqStates.delta*180/pi*ones(size(t)),'k--')
    ylabel('\delta [deg]')
    xlabel('t [s]')
    ylim([-p.deltaMax*180/pi, p.deltaMax*180/pi])
    linkaxes(ax,'x');

    
%% Global Position Trajectory
figure('Name','Trajectory E-N','Position',[1200 50 700 700]);
grid on; box on; hold on;
plot(xE,yN,'k--');
axis equal
xlabel('E [m]')
ylabel('N [m]')

% draw MARTY
for i = N+1:-1:1
    if mod(i-1,10) == 0
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

