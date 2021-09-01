function [fPts] = EstimateFeaturePoints_lalit(im,method)
%ESTIMATEFEATUREPOINTS Summary of this function goes here
%   Detailed explanation goes here
if ~exist('method','var')
    method = [1 0 0 0 0];
end
ROI = [10 10 size(im,2)-20 size(im,1)-20]; 
%%
    selec = method;
%     selec = [1 0 1 1 0]; % [Surf, kaze, fast, brisk, sift]
    names = {'surf';...
   'kaze';...
   'fast';...
   'brisk';...
   'sift'};
    selec_names = names(selec==1);
    
    % surf
    if selec(1) == 1
        kPts = detectSURFFeatures(rgb2gray(im), 'MetricThreshold', 1000, 'ROI', ROI,'NumOctaves', 12,'NumScaleLevels',6);
        [desc,kPts] = extractFeatures(rgb2gray(im), kPts, 'Method', 'Auto',...
            'Upright', true);
        loc = kPts.Location';
        fPts{1} = {loc,desc};
    else
        fPts{1}={};
    end
    
    % kaze
    if selec(2) == 1
    kPts = detectKAZEFeatures(rgb2gray(im), 'Threshold', 2e-4, 'ROI', ROI);
    [desc,kPts] = extractFeatures(rgb2gray(im), kPts, 'Method', 'Auto',...
        'Upright', false);
    loc = kPts.Location';
    fPts{2} = {loc,desc};
    else
        fPts{2}={};
    end
    
    % fast
    if selec(3) == 1
    kPts = detectFASTFeatures(rgb2gray(im), 'MinQuality', 3e-2,...
        'MinContrast', 3e-2, 'ROI', ROI);
    [desc,kPts] = extractFeatures(rgb2gray(im), kPts, 'Method', 'Auto',...
        'Upright', false);
    loc = kPts.Location';
    fPts{3} = {loc,desc.Features};
    else
        fPts{3}={};
    end
    
    % brisk
    if selec(4) == 1
    kPts = detectBRISKFeatures(rgb2gray(im), 'MinQuality', 6e-2, 'MinContrast',...
        6e-2, 'ROI', ROI);
    [desc,kPts] = extractFeatures(rgb2gray(im), kPts, 'Method', 'Auto',...
        'Upright', false);
    loc = kPts.Location';
    fPts{4} = {loc,desc.Features};
    else
        fPts{4}={};
    end
    
    % Non-native: sift
    if selec(5) == 1
    im = vl_imdown(single(rgb2gray(im)));
    [kPts,desc] = vl_sift(im,'Levels',12,'EdgeThresh',100,'Magnif',2);
    loc = 2*kPts(1:2,:);
    desc1 = imresize(desc,[64,size(desc,2)]);
    fPts{5} = {loc,desc1'};
    else
        fPts{5}={};
    end
    
    
    fPts(selec==0)=[];
    
    Loc1 = [];
    Desc1 = [];
    for i = 1:length(fPts)
        loc1 = fPts{1,i}{1,1};
        desc1 = single(fPts{1,i}{1,2});
        Loc1 = [Loc1, loc1];
        Desc1 = [Desc1; desc1];
    end
    
    fPts = {Loc1 Desc1};
    
