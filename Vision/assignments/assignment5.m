%--------------------------------------------------------------------------
%
%   assignment5.m: 3D analysis pipeline.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));
addpath(genpath('../data/'));

%% LOAD AND DISPLAY IMAGE 
% Choose which image
imNo = 2;
rgb = imread(['data/assignment5/' num2str(imNo) '_rgb.jpg']);
depth = imread(['data/assignment5/' num2str(imNo) '_depth.png']);
figure(); imshowpair(rgb,depth,'montage'); title('Original vs depth image');

%% DEPTH THRESHOLDING
if imNo == 1
    mask = depth >= 600 & depth <= 645;
elseif imNo == 2
    mask = depth >= 700 & depth <= 770;
end
% NB no image binarization needed since I work on the mask
mask = imopen(mask,strel('disk',10));
figure(); imshow(mask); title('Extracted shape');

%% SHAPE REFINEMENT - BOUNDARY & CONNECTED COMPONENTS EXTRACTION
% Improve connectivity with image opening and fill the holes
mask = bwareaopen(mask,1000);
mask = imfill(mask,'holes');
% Extract the connected components
cc = bwconncomp(mask,4);
props = regionprops('table',cc,'Area');
% The component most likely to be the object is the one with the biggest area
[~, i] = max(props.Area);
mask(:,:) = false;
mask(cc.PixelIdxList{i}) = true;
% Extract the boundary
boundary = bwboundaries(mask);
boundary = boundary{1};  
mask_boundary = false(size(mask));
for i=1:length(boundary)
    mask_boundary(boundary(i,1),boundary(i,2)) = true;
end
% Show the boundary
show_mask = zeros(size(mask,1),size(mask,2),3);
show_mask(:,:,1) = mask_boundary;
% Show the boundary
figure(); imshow(show_mask); title('Boundary');
% Show the result of the masking in color
figure(); imshow(rgb.*uint8(repmat(mask,[1 1 3]))); title('Masked image');

%% 3D POINT CLOUD
% Internal camera parameters
fu = 525; % u focal lenght in pixels
fv = 525; % v focal lenght in pixels
u0 = 319.5; % u coordinate of the principal point
v0 = 239.5; % v coordinate of the principal point
cameraParams = [fu fv u0 v0];
% Generate and plot the 3D point cloud
[cloud,cloud_rgb] = generatePointCloud(rgb,uint16(mask).*depth,cameraParams);
scatter3(cloud(:,1),cloud(:,2),cloud(:,3), 6, cloud_rgb, '.');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D point cloud'); axis equal;
% Save the result
exportMeshToPly(cloud,[],cloud_rgb,['../data/assignment5/' num2str(imNo) '_cloud']);

%% CENTROID AND ORIENTATION
cc = bwconncomp(mask, 4);
props = regionprops('table',cc,'Centroid','Orientation');
c = props.Centroid;
d = props.Orientation;
% Compute some points along the main orientation
m = -tand(d);
D = [-10:2:-1 2:2:10];
x = c(1) + D/sqrt(1+abs(m));
y = m*(x - c(1)) + c(2);
p = [x; y]';
% Plot the points on the 2D image
figure(); imshow(mask); hold on;
scatter(c(1),c(2),'r','filled');
scatter(p(:,1),p(:,2),'b','filled');
title('Main orientation'); drawnow;
% Plot the points in the 3D cloud of points
M = uint16([c; p]);
p_mask = false(size(mask));
p_mask(sub2ind(size(mask),M(:,2),M(:,1))) = true;
[orientation_cloud,~] = generatePointCloud(rgb,uint16(p_mask).*depth,cameraParams);
figure(); scatter3(cloud(:,1),cloud(:,2),cloud(:,3), 6, cloud_rgb, '.');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Main orientation'); 
hold on;
scatter3(orientation_cloud(:,1),orientation_cloud(:,2),orientation_cloud(:,3), 18, 'r', 'filled');
% Fit a line to the points, which is the main direction
[m,q] = fitLine(orientation_cloud);
% Plot the line
x = -1.25*max(cloud(:,1)):1.25*max(cloud(:,1));
L = (m.*x + q)';
hold on
scatter3(L(:,1),L(:,2),L(:,3),5,'filled'); axis equal;

%% PLANE FITTING
% Compute the plane centroid and normal
[c,n] = fitPlane(cloud);
% Plot the plane
fig8 = figure(); 
scatter3(cloud(:,1),cloud(:,2),cloud(:,3), 6, cloud_rgb, '.');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Plane fit'); axis equal;
hold on;
% Create plane mesh
[X, Y] = meshgrid(1.25*min(cloud(:,1)):8:1.25*max(cloud(:,1)), ...
    1.25*min(cloud(:,2)):8:1.25*max(cloud(:,2)));
Z = -(n(1)*X + n(2)*Y - n'*c) / n(3);
mesh(X, Y, Z, 'FaceAlpha', 0); axis equal;

%% OBJECT ORIENTATION
% The third direction is given by n x m
l = cross(n,m);
% Compute the frame by normalizing all three directions, scaled for
% visibility
O = 50.*[n/norm(n),m/norm(m),l/norm(l)];
% Plot the frame
figure(); scatter3(cloud(:,1),cloud(:,2),cloud(:,3), 6, cloud_rgb, '.');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Object orientation'); 
axis equal; hold on;
quiver3(c(1),c(2),c(3),O(1,1),O(2,1),O(3,1),'LineWidth',3,'Color','red');
quiver3(c(1),c(2),c(3),O(1,2),O(2,2),O(3,2),'LineWidth',3,'Color','green');
quiver3(c(1),c(2),c(3),O(1,3),O(2,3),O(3,3),'LineWidth',3,'Color','blue');

%% FIT BOUNDARIES
% Recover the 3D coordinates of the boundary
points = generatePointCloud(rgb,uint16(mask_boundary).*depth,cameraParams);
figure(); scatter3(cloud(:,1),cloud(:,2),cloud(:,3), 6, cloud_rgb, '.');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Boundary line fitting'); hold on;
scatter3(points(:,1),points(:,2),points(:,3),5,'filled');
% Apply ransac four times, removing each time the inliers of the previous
sampleNumber = 5;
maxIterations = 1000; 
threshold = 4;
linesM = zeros(3,4);
linesQ = zeros(3,4);
for i = 1:4
    [m,q,points] = ransacFitLine(points, ...
                                  sampleNumber, ...
                                  maxIterations, ...
                                  threshold);
    linesM(:,i) = m; linesQ(:,i) = q;
    % Plot the line
    x = -2*max(cloud(:,1)):2*max(cloud(:,1));
    L = (m.*x + q)';
    hold on; scatter3(L(:,1),L(:,2),L(:,3),5,'filled'); 
end
axis equal; ax = gca; ax.Clipping = 'off';

%% CORNER DETECTION
intersectingLines = [];
for i = 1:4
    for j = 1:4
        if j ~= i
            a = angleBetweenLines(linesM(:,i),linesM(:,j));
            if a >= 85 && a <= 95
                intersectingLines = [intersectingLines; i j];
            end
        end
    end
end   
intersectingLines = unique(sort(intersectingLines,2),'rows');
corners = [];
for i = 1:4
    q1 = linesQ(:,intersectingLines(i,1));
    m1 = linesM(:,intersectingLines(i,1));
    q2 = linesQ(:,intersectingLines(i,2));
    m2 = linesM(:,intersectingLines(i,2));
    corners = [corners; lineIntersection(q1,m1,q2,m2)'];
end
hold on;
scatter3(corners(:,1),corners(:,2),corners(:,3),35,'filled','MarkerEdgeColor','b','MarkerFaceColor','b'); 