%--------------------------------------------------------------------------
%
%   assignment4.m: Morphological operators.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));
addpath(genpath('../data/assignment4/'));

%% LOAD THE IMAGE AND CONVERT TO B/W

imageNo = 2;
I = rgb2gray(imread(strcat(num2str(imageNo),'.png')));
figure(1);
imshow(I); title('Original image'); drawnow;

%% BACKGROUND REMOVAL

% Extract the background
se = strel('disk',65);
background = imopen(I,se); 
I2 = I - background;
% Increase contrast
I3 = imadjust(I2);
% Improve shapes and get rid of holes
I3 = imclose(I3, strel('disk',25));
I3 = imfill(I3,"holes");
% Binarize so that we can use the toolbox's functions
bw = imbinarize(I3);
% Remove background noise
bw = bwareaopen(bw,50);
figure(2);
imshow(bw); title("Extracted shapes"); drawnow;

%% REGION ANALYSIS

cc = bwconncomp(bw,4);
stats = regionprops('table',bw,'Centroid','Circularity','EquivDiameter','Image','Orientation','MajorAxisLength','MinorAxisLength');
% Properties needed for shape classification
circularities = stats.Circularity;
centroids = stats.Centroid;
diameters = stats.EquivDiameter;
images = stats.Image;
orientations = stats.Orientation;
majorAxes = stats.MajorAxisLength;
minorAxes = stats.MinorAxisLength;
% Shape classification
figure(3); imshow(I); hold on;
if imageNo == 1
    % Coins and USB
    for i=1:size(circularities,1)
        if circularities(i) > 0.9
            % Coin detected
            radius = (majorAxes(i) + minorAxes(i)) / 2;
            % Is it the big coin?
            bigCoin = false;
            for j=i+1:size(circularities,1)
                if circularities(j) > 0.9
                    radius2 = (majorAxes(j) + minorAxes(j)) / 2;
                    bigCoin = radius > radius2;
                end
            end
            if bigCoin
                text(centroids(i,1),centroids(i,2),"Big coin",'Color','red');
            else
                text(centroids(i,1),centroids(i,2),"Small coin",'Color','red');
            end
        else
            % USB detected
            text(centroids(i,1),centroids(i,2),"USB stick",'Color','red');
        end
    end
else
    % Washers and bolts
    for i=1:size(circularities,1)
        if circularities(i) > 0.9
            % Washer detected, estimate the inner diameter (all numbers are empiric)
            x = (majorAxes(i) + minorAxes(i))/2;
            diameter = round((3.21821146612328e-07)*x.^5 + (-0.000132741145229905)*x.^4 + 0.0212554536472068*x.^3 + (-1.64860773332881)*x.^2 + 61.920797953864*x + (-895.382337959459));
            % Based on the diameter, get the denomination
            denomination = "Washer M" + num2str(diameter);
            text(centroids(i,1),centroids(i,2),denomination,'Color','red');
        else
            % Bolt detected, extract it as a new image and isolate the thread part
            img = images{i};
            new_angle = sign(orientations(i))*90 - orientations(i);
            img = imrotate(img, new_angle);
            img = img(int32(size(img,1)/2):size(img,1),:);
            % Based on the lenghts of the axes, get the denomination
            stats = regionprops('table',img,'MajorAxisLength','MinorAxisLength');
            % Bolt length related to the major axis
            x = stats.MajorAxisLength;
            length = round(0.00020929*x^3 - 0.0511*x^2 + 4.1439*x - 91.7938);
            % Bolt diameter related to the minor axis
            x = stats.MinorAxisLength;
            diameter = round(0.0330*x^3 - 2.1381*x^2 + 46.1296*x - 325.5931);
            denomination = "Bolt M" + num2str(diameter) + "x" + num2str(length);
            text(centroids(i,1),centroids(i,2),denomination,'Color','red');
        end
    end
end
title("Labeled image"); drawnow;
