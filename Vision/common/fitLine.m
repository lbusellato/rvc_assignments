%--------------------------------------------------------------------------
%
%   fitLine.m
%
%   This script computes the best fitting line to the given data.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [m,q] = fitLine(data)
    % The intercept is the centroid of data
    q = mean(data,1)';
    % Use PCA to compute the biggest eigenvector
    [U, E] = eig(cov(data));
    [~, i] = max(diag(E));
    % The biggest eigenvector is the direction of the line
    m = U(:,i); 
end