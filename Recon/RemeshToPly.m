function RemeshToPly(plyName,X,Normals,Colors)
%REMESHTOPLY Summary of this function goes here
%   Detailed explanation goes here
MakePly(plyName,X,Normals,Colors);
PoissonReconWrapper('out/in.ply', ['out/' plyName], 8, feature('numcores'));
fclose('all');
delete('out/in.ply')
end