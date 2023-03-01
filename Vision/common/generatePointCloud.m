%--------------------------------------------------------------------------
%
%   generatePointCloud.m
%
%   This script generates a set of 3D points based on
%   the range data contained in depthImg and a set of vertex colors based
%   on the intensity values of rgbImg. cameraParams is a vector 
%   [fu fv u0 v0], where fu=f*ku, fv=f*kv are the products of the focus 
%   length f and the focus lenghts in pixels ku and kv, u0 and v0 are 
%   the coordinates of the principal point. depthThreshold sets the 
%   maximum depth value that is considered as a valid reading.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [vertices, color] = generatePointCloud(rgbImg, depthImg, cameraParams, depthThreshold)
    assert(nargin >= 3, 'Wrong number of input arguments, check again.');
    assert(size(cameraParams,2) == 4, 'Wrong number of camera parameters, check again.');
    if nargin < 4  % If the depth threshold is not given, consider all points as valid
        depthThreshold = inf;
    end
    % Extract internal parameters
    fu = cameraParams(1,1);
    fv = cameraParams(1,2);
    u0 = cameraParams(1,3);
    v0 = cameraParams(1,4);
    [U, V] = size(depthImg);
    [u, v] = meshgrid(1:V,1:U);
    % Mask to filter out points too close or too far away
    mask = 0 < depthImg & depthImg < depthThreshold;
    % Reconstruct the 3D cloud of points
    X = (u(mask)-u0).*double(depthImg(mask))/fu;
    Y = (v(mask)-v0).*double(depthImg(mask))/fv;
    Z = double(depthImg(mask));
    vertices = [ X Y Z ];
    % Use the original RGB values to set vertex colors
    color = rgbImg(mask)./255;
end