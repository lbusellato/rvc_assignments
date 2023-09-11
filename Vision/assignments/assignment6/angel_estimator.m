%env setup
clear all
close all
addpath 'functions' 'classes';
run('functions/sift/toolbox/vl_setup');

%params
method = MethodName.Fiore;
modelFile = 'models/refDescriptorsAngel';
load(modelFile); %variable referenceModel

for i = [55,57,58,59,60,61,62,63,109,110]
figure(i);
checkImageFile = "angel/photo/angel ("+num2str(i)+").jpg";
paramsFile = "angel/visibility/angel ("+num2str(i)+").xmp";
testK = getAngelInternals(paramsFile); % estimated internal params of test image
[flag, R, T] = pose_estimator(referenceModel, checkImageFile, method, testK);
if flag
    figure(200)
    ptCloud = pcread('angel/dense/cloud.ply');
    pcshow(ptCloud)
    set(gcf,'color','w');
    set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    hold on
    plotCameraPose(referenceModel.R, referenceModel.T, '  ref');
    plotCameraPose(R, T, "  " + num2str(i));
    
    axis equal
end
end