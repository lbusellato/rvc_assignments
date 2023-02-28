%--------------------------------------------------------------------------
%
%   assignment1.m: 3rd, 5th and 7th order polynomial trajectories.
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

ti = 1; % Initial time
tf = 2; % Final time
qi = 0; % Initial position
qf = 1; % Final position
vi = 0; % Initial velocity
vf = 0; % Final velocity
ai = 0; % Initial acceleration
af = 0; % Final acceleration
ji = 0; % Initial jerk
jf = 0; % Final jerk

%% TRAJECTORY GENERATION

% 3rd order
[q3,v3,a3,j3,s3] = generatePolyTrajectory(ti, tf, qi, qf, vi, vf);
% 5th order
[q5,v5,a5,j5,s5] = generatePolyTrajectory(ti, tf, qi, qf, vi, vf, ai, af);
% 7th order
[q7,v7,a7,j7,s7] = generatePolyTrajectory(ti, tf, qi, qf, vi, vf, ai, af, ji, jf);
t = ti:0.001:tf;


%% PLOT TRAJECTORIES
fig1 = figure();
subplot(5,3,15);
% 3rd order
subplot(5,3,1); plot(t, q3, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Position"); title("3rd order");
subplot(5,3,4); plot(t, v3, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(5,3,7); plot(t, a3, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(5,3,10); plot(t, j3, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Jerk");
subplot(5,3,13); plot(t, s3, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Snap");
% 5th order
subplot(5,3,2); plot(t, q5, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Position"); title("5th order");
subplot(5,3,5); plot(t, v5, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(5,3,8); plot(t, a5, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(5,3,11); plot(t, j5, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Jerk");
subplot(5,3,14); plot(t, s5, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Snap");
% 7th order
subplot(5,3,3); plot(t, q7, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Position"); title("7th order");
subplot(5,3,6); plot(t, v7, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Velocity");
subplot(5,3,9); plot(t, a7, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Acceleration");
subplot(5,3,12); plot(t, j7, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Jerk");
subplot(5,3,15); plot(t, s7, 'LineWidth', 2); grid on; xlabel("Time [s]"); ylabel("Snap");
sgtitle("Polynomial trajectories");