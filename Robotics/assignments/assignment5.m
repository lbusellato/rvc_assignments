%--------------------------------------------------------------------------
%
%   assignment5.m: 3D trajectory
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));

%% TRAJECTORY PARAMETERS
P = [ 0 1 2 2 2;
      0 0 1 1 0;
      0 0 0 2 2]; % Path points
ti = 1; % Initial time
ts = 0.01; % Time step
dt = 1; % Duration of trajectory components

%% USING SMOOTHING CUBIC SPLINES - SAME WAYPOINTS
[spx,svx,sax,sjx,~,smx] = generateSmoothSplines(P(1,:),[0 1 2 3 4],[inf inf inf inf inf], 1);
[spy,svy,say,sjy,~,smy] = generateSmoothSplines(P(2,:),[0 1 2 3 4],[inf inf inf inf inf], 1);
[spz,svz,saz,sjz,~,smz] = generateSmoothSplines(P(3,:),[0 1 2 3 4],[inf inf inf inf inf], 1);

%% USING SMOOTHING CUBIC SPLINES - ADDED WAYPOINTS
P1 = [ 0 1 1.9 2 2 2 2;
       0 0 0.6 1 2 1 0;
       0 0  0  0 1 2 2]; % Path points
[spxp,svxp,saxp,sjxp,~,smx] = generateSmoothSplines(P1(1,:),linspace(0,4,size(P1,2)),inf*ones(1,size(P1,2)), 1);
[spyp,svyp,sayp,sjyp,~,smy] = generateSmoothSplines(P1(2,:),linspace(0,4,size(P1,2)),inf*ones(1,size(P1,2)), 1);
[spzp,svzp,sazp,sjzp,~,smz] = generateSmoothSplines(P1(3,:),linspace(0,4,size(P1,2)),inf*ones(1,size(P1,2)), 1);

%% MOTION PRIMITIVES
% For each primitive generate the parameter u and its derivatives by linear
% interpolation, then generate the motion primitive itself.
[t1,u1,uD1,uDD1,dddu1] = generateParameter(ts,ti,dt+ti,0,1,0,0);
[p1, v1, a1, j1] = rectilinear(u1, uD1, uDD1, dddu1, P(:,1), P(:,2));

[t2,u2,uD2,uDD2,dddu2] = generateParameter(ts,t1(end),t1(end)+dt,0,pi/2,0,0);
[p2, v2, a2, j2] = circular(u2, uD2, uDD2, dddu2, P(:,2), [1 1 0]', [0 0 1]');

[t3,u3,uD3,uDD3,dddu3] = generateParameter(ts,t2(end),t2(end)+dt,0,pi,0,0);
[p3, v3, a3, j3] = circular(u3, uD3, uDD3, dddu3, P(:,3), [2 1 1]', [1 0 0]');

[t4,u4,uD4,uDD4,dddu4] = generateParameter(ts,t3(end),t3(end)+dt,0,1,0,0);
[p4, v4, a4, j4] = rectilinear(u4, uD4, uDD4, dddu4, P(:,4), P(:,5));

%% 3D PLOT THE TRAJECTORY
p = [p1, p2, p3, p4];
figure();
plot3(p(1,:),p(2,:),p(3,:),'LineWidth',3); hold on;
plot3(spx,spy,spz,'LineWidth',3); hold on;
plot3(spxp,spyp,spzp,'LineWidth',3); hold on;
scatter3(spxp(1,smx),spyp(1,smy),spzp(1,smz),'ro','filled');
title('Position');
xlabel('x');
ylabel('y');
zlabel('z');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
view(-60, 30);
axis equal;
grid on;

%% PLOT POSITIONS, VELOCITIES, ACCELERATIONS, JERKS
t = [t1 t2 t3 t4];
p = [p1, p2, p3, p4];
v = [v1, v2, v3, v4];
a = [a1, a2, a3, a4];
j = [j1, j2, j3, j4];
figure();
subplot(311);
plot(t, p(1,:),'LineWidth',2); grid on; hold on;
plot(t, spx,'LineWidth',2); ylabel('Position - x');
plot(t(1,1:end-2), spxp,'LineWidth',2); ylabel('Position - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(312);
plot(t, p(2,:),'LineWidth',2); grid on; hold on;
plot(t, spy,'LineWidth',2); ylabel('Position - x');
plot(t(1,1:end-2), spyp,'LineWidth',2); ylabel('Position - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(313);
plot(t, p(3,:),'LineWidth',2); grid on; hold on;
plot(t, spz,'LineWidth',2); ylabel('Position - x');
plot(t(1,1:end-2), spzp,'LineWidth',2); ylabel('Position - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');

figure();
subplot(331);
plot(t, v(1,:),'LineWidth',2); grid on; hold on;
plot(t, svx,'LineWidth',2); ylabel('Velocity - x');
plot(t(1,1:end-2), svxp,'LineWidth',2); ylabel('Velocity - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(334);
plot(t, v(2,:),'LineWidth',2); grid on; hold on;
plot(t, svy,'LineWidth',2); ylabel('Velocity - y');
plot(t(1,1:end-2), svyp,'LineWidth',2); ylabel('Velocity - y');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(337);
plot(t, v(3,:),'LineWidth',2); grid on; hold on;
plot(t, svz,'LineWidth',2); ylabel('Velocity - z');
plot(t(1,1:end-2), svzp,'LineWidth',2); ylabel('Velocity - z');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
xlabel('Time [s]');

subplot(332);
plot(t, a(1,:),'LineWidth',2); grid on; hold on;
plot(t, sax,'LineWidth',2); ylabel('Acceleration - x');
plot(t(1,1:end-2), saxp,'LineWidth',2); ylabel('Acceleration - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(335);
plot(t, a(2,:),'LineWidth',2); grid on; hold on;
plot(t, say,'LineWidth',2); ylabel('Acceleration - y');
plot(t(1,1:end-2), sayp,'LineWidth',2); ylabel('Acceleration - y');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(338);
plot(t, a(3,:),'LineWidth',2); grid on; hold on;
plot(t, saz,'LineWidth',2); ylabel('Acceleration - z');
plot(t(1,1:end-2), sazp,'LineWidth',2); ylabel('Acceleration - z');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
xlabel('Time [s]');

subplot(333);
plot(t, j(1,:),'LineWidth',2); grid on; hold on;
plot(t, sjx,'LineWidth',2); ylabel('Jerk - x');
plot(t(1,1:end-2), sjxp,'LineWidth',2); ylabel('Jerk - x');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(336);
plot(t, j(2,:),'LineWidth',2); grid on; hold on;
plot(t, sjy,'LineWidth',2); ylabel('Jerk - y');
plot(t(1,1:end-2), sjyp,'LineWidth',2); ylabel('Jerk - y');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
subplot(339);
plot(t, j(3,:),'LineWidth',2); grid on; hold on;
plot(t, sjz,'LineWidth',2); ylabel('Jerk - z');
plot(t(1,1:end-2), sjzp,'LineWidth',2); ylabel('Jerk - z');
legend('Primitives', 'Smoothing', 'Smoothing - more waypoints', 'Location','best');
xlabel('Time [s]');