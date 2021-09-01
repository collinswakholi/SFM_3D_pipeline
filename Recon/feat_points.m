
% clear all; close all; clc;
function fdata = feat_point(II,show)

if nargin < 2
    showIm = 0;
elseif nargin
    showIm = show;
end

cmn_num = [0 1 1 1 0];
names = {'kaze';...
   'fast';...
   'surf';...
   'brisk';...
   'harris'};
selected_features = (names(cmn_num==1));


I = rgb2gray(II);

        points{1} = detectKAZEFeatures(I);
        points{2} = detectFASTFeatures(I);
        points{3} = detectSURFFeatures(I);
        points{4} = detectBRISKFeatures(I);
        points{5} = detectHarrisFeatures(I);
        points(cmn_num==0)=[];

        len = length(points);
        location = [];
        for i =1:len
            location = [location; points{1, i}.Location];
        end
fdata = location;

%% show image
if showIm ~=0
    imshow(II)
    hold on

        scatter(fdata(:,1),fdata(:,2),'+g')

    hold off
end