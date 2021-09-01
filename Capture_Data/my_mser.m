% mser to detect squared checkers
% I = rgb2gray(imread('out\2017_1113_Warm carcass 1_Left_Camera1_1.png'));
I = (imread('out\2017_1113_Warm carcass 1_Left_Camera1_1.png'));
Ir = I(:,:,1);
sz = size(Ir);

I1 = I(:,(round(sz(2)/2):end),:);

r = vl_mser(I1,'MinDiversity',0.2,'MaxVariation',0.02,'MaxArea',0.01) ;

M_1 = zeros(size(I1)) ;
for x=r'
  s = vl_erfill(I1,x) ;
  M_1(s) = M_1(s) + 1;
end

BW = bwareaopen(M_1,100);
stats = regionprops(BW,'BoundingBox','PixelIdxList','Area');

Area = [stats.Area];

BB = [];
for i = 1:length(stats)
    bb = stats(i).BoundingBox;
    BB = [BB;bb];
end

[N,edges] = histcounts(BB(:,1));
[~,mx] = max(N);
lim = [edges(mx),edges(mx+1)];
idx1 = find((abs(BB(:,3)-BB(:,4))<10)&...
    ((BB(:,1)>lim(1))&(BB(:,1)<lim(2)))); % bounding box size and location

[N1,edges1] = histcounts(Area);
[~,mx1] = max(N1);
tol1 = round((edges(2)-edges(1))/4);

lim1 = [edges1(mx1)-tol,edges1(mx1)+tol];

idx2 = find(Area>lim1(1) & Area<lim1(2)); % Area

BB1 = BB(idx1,:);
Area1 = Area(idx1);
stats1 = stats(idx1);

Z = linkage([BB1(:,3),BB1(:,end),Area1'],'ward');
c = cluster(Z,'Maxclust',3);
idx2 = find(c == mode(c));

scatter3(BB1(:,3),BB1(:,end),Area1',10,c)

BB2 = BB1(idx2,:);
Area2 = Area1(idx2);
stats2 = stats1(idx2);

imshow(I1);
BBw = false(size(BW));
for ii = 1:length(idx1)
    rectangle('Position',BB1(ii,:),'EdgeColor','g','LineWidth',2)
    hold on;
    
    BBw(stats2(ii).PixelIdxList) = true;
end

imshow(I)
hold on ;
h1 = vl_plotframe(fff) ; set(h1,'color','g','linewidth',3) ;
h2 = vl_plotframe(fff) ; set(h2,'color','k','linewidth',1) ;
vl_demo_print('mser_basic_frames_both') ;

figure(2) ;
if vl_isoctave()
  [c,h]=contour(M,(0:max(M(:)))+.5,'y','linewidth',3) ;
else
  [c,h]=contour(M,(0:max(M(:)))+.5) ;
  set(h,'color','g','linewidth',3) ;
end