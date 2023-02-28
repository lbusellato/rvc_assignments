%--------------------------------------------------------------------------
%
%   assignment6.m: 3D trajectory along a sphere.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));

%% SPHERE GENERATION
% Sphere center - random
P0 = randi(10, 1, 3); 
% Sphere radius - random
R = randi(10, 1, 1) + 1;
% Generate the sphere's coordinates
S = generateSphere(P0, R, 50);
surf(S.X, S.Y, S.Z); 
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Geodesic curves - Frenet frames'); 
colormap bone; axis equal; hold on;
% Pick n random points
i = randi([1 size(S.X, 1)], 1, 3);
j = randi([1 size(S.X, 1)], 1, 3);
P = [S.X(i(1), j(1)) S.X(i(2), j(2)) S.X(i(3), j(3));
     S.Y(i(1), j(1)) S.Y(i(2), j(2)) S.Y(i(3), j(3));
     S.Z(i(1), j(1)) S.Z(i(2), j(2)) S.Z(i(3), j(3))];
% Plot the points
scatter3(P(1, :), P(2, :), P(3, :),'r','filled'); hold on;
ax = gca; ax.Clipping = 'off';

%% GEODESICS & FRENET FRAMES
[G1,t1,n1,b1] = geodesic(P(:, 1), P(:, 2), P0', pi / 24);
[G2,t2,n2,b2] = geodesic(P(:, 1), P(:, 3), P0', pi / 24);
[G3,t3,n3,b3] = geodesic(P(:, 3), P(:, 2), P0', pi / 24);
G = [G1; G2; G3];
scatter3(G(:,1),G(:,2),G(:,3),'.b'); hold on;
for i=1:size(t1,2)
     quiver3(G1(i,1),G1(i,2),G1(i,3),t1(1,i),t1(2,i),t1(3,i),'Color','red','LineWidth',2)
     quiver3(G1(i,1),G1(i,2),G1(i,3),b1(1,i),b1(2,i),b1(3,i),'Color','green','LineWidth',2)
     quiver3(G1(i,1),G1(i,2),G1(i,3),n1(1,i),n1(2,i),n1(3,i),'Color','blue','LineWidth',2)
end
for i=1:size(t2,2)
     quiver3(G2(i,1),G2(i,2),G2(i,3),t2(1,i),t2(2,i),t2(3,i),'Color','red','LineWidth',2)
     quiver3(G2(i,1),G2(i,2),G2(i,3),b2(1,i),b2(2,i),b2(3,i),'Color','green','LineWidth',2)
     quiver3(G2(i,1),G2(i,2),G2(i,3),n2(1,i),n2(2,i),n2(3,i),'Color','blue','LineWidth',2)
end
for i=1:size(t3,2)
     quiver3(G3(i,1),G3(i,2),G3(i,3),t3(1,i),t3(2,i),t3(3,i),'Color','red','LineWidth',2)
     quiver3(G3(i,1),G3(i,2),G3(i,3),b3(1,i),b3(2,i),b3(3,i),'Color','green','LineWidth',2)
     quiver3(G3(i,1),G3(i,2),G3(i,3),n3(1,i),n3(2,i),n3(3,i),'Color','blue','LineWidth',2)
end