function [P, X] = EstimateRealPose(E, corrsNorm, CoordSys)
%ESTIMATEREALPOSE Summary of this function goes here
%   Detailed explanation goes here
if ~exist('CoordSys','var')
    CoordSys = CANONICAL_POSE;
end
CP = EstimateAmbiguousPose(E);

X1 = Triangulate(CANONICAL_POSE, CP{1}, corrsNorm);
X2 = Triangulate(CANONICAL_POSE, CP{2}, corrsNorm);
X3 = Triangulate(CANONICAL_POSE, CP{3}, corrsNorm);
X4 = Triangulate(CANONICAL_POSE, CP{4}, corrsNorm);

[P, X] = DisambiguateCameraPose(CP, {X1, X2, X3, X4});

%P_DEP = P*[CoordSys; 0 0 0 1];

R_ = CoordSys(1:3,1:3);
t_ = CoordSys(:,end);
R__ = P(1:3,1:3);
t__ = P(:,end);

P = [R__*R_  R__*t_+t__];

end

