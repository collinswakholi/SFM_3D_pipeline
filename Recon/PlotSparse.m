function figur = PlotSparse(CP, X, C)
%PLOTSPARSE Summary of this function goes here
%   Detailed explanation goes here
CR = cellfun(@(P) P(1:3,1:3), CP, 'UniformOutput', false);
CC = cellfun(@(P) -P(1:3,1:3)'*P(1:3,4), CP, 'UniformOutput', false);
if exist('X','var'),figur = Display3D(CC, CR, X, C);else,figur = Display3D(CC,CR);end
drawnow
end

