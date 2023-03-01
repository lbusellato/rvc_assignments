%--------------------------------------------------------------------------
%
%   point2lineDistance.m
%
%   This script computes the distance of a point from a line.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [d2H,H] = point2lineDistance(M,C,D)
    h = cross(repmat(D',[size(M,1),1]),(M-C'),2);
    d2H = vecnorm(h,2,2);  % distance
    H = M + cross(repmat(D',[size(M,1),1]),h,2);  % projection
end
