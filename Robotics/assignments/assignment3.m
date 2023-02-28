%--------------------------------------------------------------------------
%
%   assignment3.m: Interpolating polynomials
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));

%% CUBIC SPLINES W/ INITIAL|PATH|FINAL VELOCITIES
qk = [10 20 0 30 40]; % Path points
tk = [0 2 4 8 10]; % Times
dqk = [0 0 0 5.2 0]; % Path point velocities

[p,v,a,j,t,m] = generateCubicSplines(qk,tk,dqk,false);

%% DISPLAY TRAJECTORY
fig1 = figure();
subplot(411); plot(t, p,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Position");
subplot(412); plot(t, v,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(413); plot(t, a,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(414); plot(t, j,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Jerk");
sgtitle("CUBIC SPLINES WITH GIVEN INITIAL|PATH|FINAL VELOCITIES");

%% CUBIC SPLINES W/ INITIAL|FINAL VELOCITIES
qk = [10 20 0 30 40]; % Path points
tk = [0 2 4 8 10]; % Times
dqk = [0 0]; % Path point velocities

[p,v,a,j,t,m] = generateCubicSplines(qk,tk,dqk,false);

%% DISPLAY TRAJECTORY
fig2 = figure();
subplot(411); plot(t, p,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Position");
subplot(412); plot(t, v,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(413); plot(t, a,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(414); plot(t, j,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Jerk");
sgtitle("CUBIC SPLINES WITH GIVEN INITIAL|FINAL VELOCITIES");

%% CUBIC SPLINES W/ INITIAL|FINAL VELOCITIES AND CONTINUOUS ACCELERATIONS
qk = [10 20 0 30 40]; % Path points
tk = [0 2 4 8 10]; % Times
dqk = [0 0];

[p,v,a,j,t,m] = generateCubicSplines(qk,tk,dqk, true);

%% DISPLAY TRAJECTORY
fig2 = figure();
subplot(411); plot(t, p,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Position");
subplot(412); plot(t, v,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(413); plot(t, a,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(414); plot(t, j,'Marker','o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; xlabel("Time [s]"); ylabel("Jerk");
sgtitle("CUBIC SPLINES WITH GIVEN INITIAL|FINAL VELOCITIES & CONTINUOUS ACCELERATION");
