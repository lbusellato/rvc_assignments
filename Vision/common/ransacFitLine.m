%--------------------------------------------------------------------------
%
%   ransacFitLine.m
%
%   This script implements RANSAC to robustly estimate a line fitting the
%   given data.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [m,q,outliers] = ransacFitLine(data, sampleNumber, maxIterations, threshold)
    inlier_max = 0;
    for it = 1:maxIterations 
        % Select n random elements from data
        ids = ceil(rand(1,sampleNumber)*size(data,1));
        % Fit a line on them
        [mi,qi] = fitLine(data(ids,:));
        % Count inliers
        d = point2lineDistance(data,qi,mi);
        inlier_cnt = sum(d<threshold);
        if inlier_cnt > inlier_max % Better than the best model so far
            inlier_max = inlier_cnt;
            m = mi; q = qi;
            outliers = data(point2lineDistance(data,q,m) >= threshold,:);
        end            
    end    
end