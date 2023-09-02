addpath 'functions' 'classes';
addpath(genpath('./angel/'))
run('functions/sift/toolbox/vl_setup');

env = 'dante'; %dante or cav
disp('Loading points...');
% Load reference camera info
imageIndex = '1020';
[K_ref, R_ref, T_ref, p2D, p3D] = dante_get_points('angel/visibility/SamPointCloud.ply', ...
    "angel/VisibilityRef.txt", ...
    "angel/visibility/angel (111).xmp");
refImg = imread("angel/photo/angel (111).jpg");

nPoint = length(p3D);
fprintf('Found %i points\n',nPoint);
disp('Building descriptors...');
[f, d] = vl_sift(single(rgb2gray(refImg))) ;
[sel, dist] = dsearchn(f(1:2,:)',p2D);
threshold = 4; 
valid = dist < threshold;
sel = sel(valid);

[p2D_ref, p3D_ref, f_ref, d_ref] = getRefDescriptors(p2D, p3D, f(:,sel), d(:,sel));

fprintf('Attached descriptors to %i points\n', length(p2D_ref));

fileName = 'models/refDescriptorsAngel.mat';
referenceModel = ReferenceModel(refImg, p2D_ref, p3D_ref, K_ref, R_ref, T_ref, f_ref, d_ref);
save(fileName, 'referenceModel');
fprintf('Saved model in %s\n', fileName);