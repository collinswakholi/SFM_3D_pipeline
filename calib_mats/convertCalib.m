clear; clc;

name = 'calibrationSession'; % name of calib file
KK = [];
dd= [];
for i = 1
    load([name,num2str(i),'.mat']);
    % delete([name,num2str(i),'.mat']);
    k = calibrationSession.CameraParameters.IntrinsicMatrix;
    K = k';
    d1 = calibrationSession.CameraParameters.RadialDistortion;
    d = d1(1:2);
    KK = cat(3,KK,K);
    dd = cat(3,dd,d);
end
K = geomean(KK,3);
d = mean(dd,3);
clearvars -except K d
save('calib_new.mat')


