function [X,Colors,ScreenedX,ScreenedColors,Disparity] =...
    RectifyAndDenseTriangulate(zz,im1,im2,F,K,P1,P2,zRange,denoiseFlag,CplotFlag)
%RECTIFYANDDENSETRIANGULATE Summary of this function goes here
%   Detailed explanation goes here

[H1, H2, im1Rec, im2Rec] = rectify(F,im1,im2);
im1RecPlt = im1Rec;
im2RecPlt = im2Rec;
im1Rec(im1Rec==0) = -1;
im2Rec(im2Rec==0) = -9999;
KP1Rec = H1*K*P1;
KP2Rec = H2*K*P2;

pars = [];
% pars.mu = -10.6;
% pars.window = 4;
% pars.zonegap = 7;
% pars.pm_tau = 0.99;
% pars.tau = 0.7;
pars.mu = zz(1);
pars.window = zz(2);
pars.zonegap = zz(3);
pars.pm_tau = zz(4);
pars.tau = zz(5);

Disparity = gcs(im1Rec, im2Rec, [], pars);
save 

if all(all(isnan(Disparity)))
    imshow(imfuse(im1RecPlt,im2RecPlt))
    error('No matches found')
end

if exist('CplotFlag','var')
    if any(contains(CplotFlag, 'plotCorrespondences'))
        mc12Rec = CorrsFromDisparity(Disparity);
        g = fix(linspace(1, size(mc12Rec,2), 1000));
        PlotCorrespondences(im1RecPlt, im2RecPlt, ...
            mc12Rec(:,g), mc12Rec(:,g));
        drawnow
    end
    if any(contains(CplotFlag, 'plotDisparityMap'))
        figure
        set(imagesc(Disparity),'AlphaData',~isnan(Disparity)), colormap(jet)
        colorbar,drawnow
    end
end

if strcmp(zRange,'manual')
    h = figure('pos', [0 100 getfield(get(0,'screensize'),{3}) 300]);
    set(0,'CurrentFigure',h)
    histogram(Disparity,1000), drawnow
    lb = ginput(1);
    ub = ginput(1);
    hold on, plot([lb(1) ub(1)], [0 0], 'rx', 'LineWidth', 2), drawnow
    hold off, close(gcf)
    Disparity(Disparity<lb(1) | Disparity>ub(1)) = nan;
elseif strcmp(zRange,'auto')
end
mc12Rec = CorrsFromDisparity(Disparity);

X = Triangulate(KP1Rec, KP2Rec, mc12Rec);
if numel(zRange) == 2
    X_ = X;
    
    X = Triangulate(H1*K*CANONICAL_POSE, H2*K*DehomogenizeMat(HomogenizeMat(P2)/...
        HomogenizeMat(P1)), mc12Rec);    
    inInd = X(3,:) > zRange(1) & X(3,:) < zRange(2);
    NotNanInd = ~isnan(Disparity);
    NotNanAndNotBackgroundInd = false(size(Disparity));
    NotNanAndNotBackgroundInd(NotNanInd==true) = inInd;
    mc12Rec = mc12Rec(:,inInd);
    Aux = nan(size(Disparity));
    Aux(NotNanAndNotBackgroundInd) = Disparity(NotNanAndNotBackgroundInd);
    Disparity = Aux;
    
    X = X_;
    X = X(:,inInd);
end

sz = [size(im1,1) size(im1,2)];
mc1Unr = [fix(Dehomogenize(H1\Homogenize(mc12Rec(1:2,:))));
    nan(1,size(mc12Rec,2))];

Colors = bsxfun(@(x,dummy) ...
    double(permute(im1(...
    min(max(x(2),1), sz(2)),...
    min(max(x(1),1), sz(1)),...
    :),[3 2 1])), ...
    mc1Unr, 1:size(mc12Rec,2));

if exist('denoiseFlag','var') && strcmp(denoiseFlag,'denoise')
    [~,inInd] = pcdenoise(pointCloud(X'), 'NumNeighbors', 100, 'Threshold', 0.1);
    inInd = ismember(1:size(X,2), inInd);
    X = X(:,inInd);
    Colors = Colors(:,inInd);
    
    NotNanInd = ~isnan(Disparity);
    NotNanAndNotNoiseInd = false(size(Disparity));
    NotNanAndNotNoiseInd(NotNanInd==true) = inInd;
    Ind = NotNanAndNotNoiseInd;
else
    Ind = ~isnan(Disparity);
end

    ScreenedX1 = nan(size(Disparity));
    ScreenedX1(Ind) = X(1,:);
    ScreenedX2 = nan(size(Disparity));
    ScreenedX2(Ind) = X(2,:);
    ScreenedX3 = nan(size(Disparity));
    ScreenedX3(Ind) = X(3,:);
    ScreenedX(:,:,1) = ScreenedX1;
    ScreenedX(:,:,2) = ScreenedX2;
    ScreenedX(:,:,3) = ScreenedX3;

    ScreenedColors1 = nan(size(Disparity));
    ScreenedColors1(Ind) = Colors(1,:);
    ScreenedColors2 = nan(size(Disparity));
    ScreenedColors2(Ind) = Colors(2,:);
    ScreenedColors3 = nan(size(Disparity));
    ScreenedColors3(Ind) = Colors(3,:);
    ScreenedColors(:,:,1) = ScreenedColors1;
    ScreenedColors(:,:,2) = ScreenedColors2;
    ScreenedColors(:,:,3) = ScreenedColors3;

end