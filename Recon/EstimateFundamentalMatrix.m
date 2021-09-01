function [ CF ] = EstimateFundamentalMatrix( Cx, algorithm )
%ESTIMATEFUNDAMENTALMATRIX Summary of this function goes here
%   Detailed explanation goes here
if ~exist('algorithm','var')
    algorithm = 'Norm8PtAlg';
end

x = cell2mat(Cx);
x1 = x(1:2,:);
x2 = x(3:4,:);

if strcmp(algorithm, 'Norm8PtAlg')
    F = EstimateFundamentalMatrixNorm8PtAlg(x1, x2);
    F = F/norm(F);
    CF = {F};
elseif strcmp(algorithm, '8PtAlg')
    F = EstimateFundamentalMatrix8PtAlg(x1, x2);
    F = F/norm(F);
    CF = {F};
else
    error(['Algorithm ' algorithm ' does not exist'])
end
end

function F = EstimateFundamentalMatrixNorm8PtAlg(x1, x2)
[x1,T1] = normalizePoints(x1);
[x2,T2] = normalizePoints(x2);

F = EstimateFundamentalMatrix8PtAlg(x1, x2);

F = [T2; [0 0 1]]'*F*[T1; [0 0 1]];
end

function F = EstimateFundamentalMatrix8PtAlg(x1, x2)
if length(x1) < 8
    error('At least 8 points required for Fundamental Matrix estimation')
end

A = [x2(1,:)'.*x1(1,:)' x2(1,:)'.*x1(2,:)' x2(1,:)'...
    x2(2,:)'.*x1(1,:)' x2(2,:)'.*x1(2,:)' x2(2,:)'...
    x1(1,:)' x1(2,:)' ones(size(x1,2),1)];

[~,~,V] = svd(A,0);
F = reshape(V(:,9),3,3)';
[U,D,V] = svd(F);
F = U*diag([D(1,1) D(2,2) 0])*V';
end

function [xNorm, T] = normalizePoints(x)
centroid = mean(x,2);
meanDist = mean(sqrt((x(1,:)-centroid(1)).^2 + (x(2,:)-centroid(2)).^2));
scale = sqrt(2)/meanDist;

T = [scale 0 -scale*centroid(1);
    0 scale -scale*centroid(2)];
xNorm = T*[x; ones(1,size(x,2))];
end