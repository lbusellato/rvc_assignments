%--------------------------------------------------------------------------
%
%   circular.m
%
%   This script generates the position, velocity, acceleration and jerk
%   profiles for a circular motion primitive with center c, axis of 
%   rotation r, starting from point P and parametrized wrt u.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [q,dq,ddq,dddq] = circular(u,du,ddu,dddu,P,c,r)
    x = P - c;
    y = cross(r,x);
    R = [x y r];
    ro = norm(x);
    q = c + R*[ro*cos(u);ro*sin(u); zeros(size(u))];
    dq = R*(du.*[-ro*sin(u); ro*cos(u); zeros(size(u))]);
    ddq = R*(ddu.*[-ro*sin(u); ro*cos(u); zeros(size(u))] ...
           + du.*[-ro*cos(u); -ro*sin(u); zeros(size(u))]);
    dddq = R*(dddu.*[-ro*sin(u); ro*cos(u); zeros(size(u))] ...
             + 2 * ddu.*[-ro*cos(u);-ro*sin(u); zeros(size(u))] ...
           + du.*[ro*sin(u); -ro*cos(u); zeros(size(u))]);
end