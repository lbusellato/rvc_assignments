%--------------------------------------------------------------------------
%
%   assignment2.m: Trapezoidal trajectory.
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

ti = 0; % Initial time
tf = 5; % Final time
qi = 0; % Initial position
qf = 5; % Final position
tc = 2; % Acceleration/deceleration time                                  
vc = nan; % Velocity in the costant velocity phase                          
ac = nan; % Acceleration in the linear velocity phase
dqi = nan; % Initial velocity
dqf = nan; % Final velocity
am = nan; % Maximum acceleration
vm = nan; % Maximum velocity

%params = [ti, tf, qi, qf, tc, vc, ac, dqi, dqf, am, vm];
% params = [0 5 0 5 2 nan nan nan nan nan nan]; % tc fixed
% params = [0 5 5 0 2 nan nan nan nan nan nan]; % tc fixed, qf<qi
% params = [1 5 0 5 2 nan nan nan nan nan nan]; % tc fixed, ti!=0
% params = [0 5 0 5 nan 2 nan nan nan nan nan]; % vc fixed, triangular vel
% params = [0 5 0 5 nan 1.5 nan nan nan nan nan]; % vc fixed, trapezoidal vel
% params = [0 5 0 5 nan nan 2 nan nan nan nan]; % ac fixed
% params = [0 5 0 5 nan 2 2 nan nan nan nan]; % vc and ac fixed
 params = [0 5 5 0 nan nan nan 0.5 1 2 nan]; % dqi dqf and ddqmax fixed
% params = [1 5 5 -5 nan nan nan -3 2 3 4]; % dqi dqf ddqmax and dqmax fixed

%% TRAPEZOIDAL TRAJECTORY GENERATION
[q,v,a,t] = generateTrapezoidal(params(1),params(2),params(3), ...
                                params(4),params(5),params(6), ...
                                params(7),params(8),params(9), ...
                                params(10),params(11));
subplot(311); grid on; plot(t,q,'LineWidth',2); ylabel('Position'); grid on;
subplot(312); grid on; plot(t,v,'LineWidth',2); ylabel('Velocity'); grid on;
subplot(313); grid on; plot(t,a,'LineWidth',2); ylabel('Acceleration'); xlabel('Time [s]'); grid on;

%% MULTIPOINT TRAJECTORY
[q,v,a,t,m] = multipointTrapezoidal([0 20 50 40 10], 0, 0, 15, 20, true);

figure();
subplot(311); plot(t,q,'-o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; ylabel('Position'); grid on;
subplot(312); plot(t,v,'-o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; ylabel('Velocity'); grid on;
subplot(313); plot(t,a,'-o','MarkerIndices',m,'MarkerEdgeColor','r','LineWidth',2); grid on; ylabel('Acceleration'); xlabel('Time [s]'); grid on;