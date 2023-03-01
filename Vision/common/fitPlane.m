%--------------------------------------------------------------------------
%
%   fitPlane.m
%
%   This script computes the best fitting plane to the given data.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [c,n] = fitPlane(data)
    % Plane centroid
    c = mean(data,1)'; 
    % Use PCA to compute the smallest eigenvector
    [U, E] = eig(cov(data));  
    [~, i] = min(diag(E)); 
    % The smallest eigenvector is the surface normal
    n = U(:,i); 
end