%--------------------------------------------------------------------------
%
%   generateSphere.m
%
%   This script computes the x,y,z coordinates of a sphere of given center
%   and radius, approxymated to n polygonal faces per
%   longitudinal/latitudinal slice.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function S = generateSphere(P0, R, n)
    [X1, Y1, Z1] = sphere(n);
    S.X = (R*X1)+P0(1,1);
    S.Y = (R*Y1)+P0(1,2);
    S.Z = (R*Z1)+P0(1,3);
end