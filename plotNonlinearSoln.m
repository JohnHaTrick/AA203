function plotNonlinearSoln( t, sol, p)
%PLOTNONLINEARSOLN
%   Plot solution of AA203 nonlinear trajectory

%% Parse Solution Data
N = length(t)-1;
% State Variables
xE          = sol.state.xE;
yN          = sol.state.yN;
Psi         = sol.state.Psi;
Ux          = sol.state.Ux;
Uy          = sol.state.Uy;
r           = sol.state.r;
% Input Variables
% Tr          = sol.input.Tr;
Tr          = sol.input.Fxr*p.Rwr;
delta       = sol.input.delta;


%% State Variables
figure('Name','State Variables','Position',[0 0 400 1000])
subplot(611); hold on; box on
    plot(t,xE,'k'); grid on
    ylabel('E [m]')
    set(gca,'xticklabel',{})
subplot(612); hold on; box on
    plot(t,yN,'k'); grid on
    ylabel('N [m]')
    set(gca,'xticklabel',{})
subplot(613); hold on; box on
    plot(t,Psi/pi*180,'k'); grid on
    set(gca,'xticklabel',{})
    ylabel('\Psi [deg]')
subplot(614); hold on; box on
    plot(t,Ux,'k'); grid on
    ylabel('U_x [m/s]')
    set(gca,'xticklabel',{})
subplot(615); hold on; box on
    plot(t,Uy,'k'); grid on
    ylabel('U_y [m/s]')
    set(gca,'xticklabel',{})
subplot(616); hold on; box on
    plot(t,r,'k'); grid on
    ylabel('r [rad/s]')
    xlabel('t [s]')

    
%% Input Variables
figure('Name','Input Variables','Position',[400 0 400 600])
subplot(211); hold on; box on
    stairs(t(1:N),Tr,'k'); grid on
    ylabel('T_r [Nm]')
    ylim([p.Tmin, p.Tmax])
    set(gca,'xticklabel',{})
    subplot(212); hold on; box on
    stairs(t(1:N),delta/pi*180,'k'); grid on
    ylabel('\delta [deg]')
    xlabel('t [s]')
    ylim([-p.deltaMax*180/pi, p.deltaMax*180/pi])

    
%% Global Position Trajectory
figure('Name','Trajectory E-N','Position',[800 400 400 400])
plot(xE,yN,'k'); grid on; box on
axis equal
xlabel('E [m]')
ylabel('N [m]')


%% what's this do?
GenerateFig = false;
if GenerateFig
    fileName = 'ICE2018';
    mlf2pdf(gcf,fileName)
    system(strcat('pdfcrop --noverbose ./',fileName,'.pdf ./', fileName,'.pdf'));
end

end

