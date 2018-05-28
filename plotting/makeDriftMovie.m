%% makeDriftMovie

% Use data present in workspace after running SimAndOptHarness to create a
% movie for AA203 presentation

close all;

%% Parse Solution Data
t           = interpResults.t;
dt          = interpResults.dt;
N           = size(t,2)-1;
% State Variables
xE          = interpResults.xE;
yN          = interpResults.yN;
Psi         = interpResults.Psi;
Ux          = interpResults.Ux;
Uy          = interpResults.Uy;
r           = interpResults.r;
% Input Variables
Tr          = interpResults.Tr;
delta       = interpResults.delta;

%% set up figure dimensions

figPos = [10,50, 1600, 800];
lowLim = .07;
leftLim = .05;

fig = figure('Position', figPos );

steer = subplot('Position', [leftLim,0.55,0.45,0.4] ); % upper left
    grid on;
    title('Steering Angle')
%     xlabel('time [sec]')
    ylabel('[deg]')
    xlim manual; xlim([t(1),floor(t(end)*2/3)])
    ylim manual; ylim([-vehicle.deltaMax*180/pi,vehicle.deltaMax*180/pi])

torque = subplot('Position', [leftLim,lowLim,0.45,0.4] ); % lower right
    grid on;
    title('Rear Axle Torque')
    xlabel('time [sec]')
    ylabel('[kN-m]')
    xlim manual; xlim([t(1),floor(t(end)*2/3)])
    ylim manual; ylim([-5,5])

state = subplot('Position', [0.55,lowLim,0.4,0.88] ); % right side
    grid on; axis equal;
    title('Gobal Position')
    xlabel('East [m]')
    ylabel('North [m]')
    xlim manual; xlim([min(xE)-15,max(xE)+15])
    ylim manual; ylim([yN(1)-10,yN(end)+20])
    % plot target circle
    r = p.R;
    target = rectangle('Position',[0-2*r, p.N_f-r, 2*r, 2*r],'Curvature',[1 1],'LineStyle','--','EdgeColor','r');


%% plot state and inputs
% vehicle params
wid = 1.6;    len = 2.2;    trac = 1;

% movie data
n = 5; % data skips
% Frames = getframe(fig);
v = VideoWriter('driftVid.avi');
v.FrameRate = 1/dt(1)/n;
open(v);

for i = 1:length(t)
    
    if mod(i,n)
        j = min(i,length(t)-1);

        axes(steer); hold on;
            plot(t(1:j),delta(1:j).*(180/pi),'r','LineWidth',2);

        axes(torque); hold on;
            plot(t(1:j),Tr(1:j)./1000,'r','LineWidth',2);

        axes(state); hold on;
    %         disp(target);
            R   = plot([xE(i)-wid/2, xE(i)+wid/2], ...
                       [yN(i)-len/2, yN(i)-len/2],'Color',[.5 .5 .5],'LineWidth',2);
            F   = plot([xE(i)+wid/2, xE(i)-wid/2], ...
                       [yN(i)+len/2, yN(i)+len/2],'Color',[.5 .5 .5],'LineWidth',2);
            C   = plot([xE(i),     xE(i)], ...
                       [yN(i)-len/2, yN(i)+len/2],'Color',[.5 .5 .5],'LineWidth',2);
            Wrr = plot([xE(i)+wid/2, xE(i)+wid/2], ...
                       [yN(i)-len/2-trac/2, yN(i)-len/2+trac/2],'k','LineWidth',3);
            Wrl = plot([xE(i)-wid/2, xE(i)-wid/2], ...
                       [yN(i)-len/2-trac/2, yN(i)-len/2+trac/2],'k','LineWidth',3);
            Wfr = plot([xE(i)+wid/2, xE(i)+wid/2], ...
                       [yN(i)+len/2-trac/2, yN(i)+len/2+trac/2],'k','LineWidth',3);
            Wfl = plot([xE(i)-wid/2, xE(i)-wid/2], ...
                       [yN(i)+len/2-trac/2, yN(i)+len/2+trac/2],'k','LineWidth',3);
            rotate(Wfr, [0 0 1], delta(min(i,N))*180/pi, [xE(i)+wid/2,yN(i)+len/2,0]);
            rotate(Wfl, [0 0 1], delta(min(i,N))*180/pi, [xE(i)-wid/2,yN(i)+len/2,0]);
            rotate(Wrr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(Wrl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(Wfr, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(Wfl, [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(F,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(R,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            rotate(C,   [0 0 1], Psi(i)*180/pi, [xE(i),yN(i),0]);
            refreshdata
            drawnow
    %         Frames = [Frames, getframe(fig)];
        writeVideo(v,getframe(fig));
        if i ~= length(t)
            delete([R,F,C,Wrr,Wrl,Wfr,Wfl]);
        end        
    end
end

% movie(fig,Frames)
close(v)