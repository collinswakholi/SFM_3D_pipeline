function [matchesCoords] = MatchFeaturePoints(fPts1,fPts2,method,maxth)
%MATCHFEATUREPOINTS Summary of this function goes here
%   Detailed explanation goes here
if ~exist('method','var')
    method = 'Exhaustive';
end

if ~exist('maxth','var')
    method = 0.6;
end

i = matchFeatures(fPts1{2}, fPts2{2}, 'Method', method, 'Unique', true,...
        'MaxRatio', maxth,'MatchThreshold', 30);% 'MatchThreshold', 30
    matchesCoords = double([fPts1{1}(:,i(:,1)); fPts2{1}(:,i(:,2))]);
end

