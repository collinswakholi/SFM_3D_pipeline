function ptcd_selec = selectPointColud (ptcd,labels,numClusters,selec)

assert(selec<=numClusters,'Reduce selected cluster index');

selected_idx = (labels>(selec-0.5))&(labels<(selec+0.5));
sum(selected_idx)

[nloc,loc] = deal(ptcd.Location);
[ncol,col] = deal(ptcd.Color);
[nnor,nor] = deal(ptcd.Normal);

idx = find(selected_idx~=1);

nloc(idx,:)=[];

try
    ncol(idx,:)=[];
catch
    ncol = [];
end

try
    nnor(idx,:)=[];
catch
    nnor = [];
end

ptcd_selec = pointCloud(nloc, 'Color',ncol,'Normal',nnor);

pcshow(ptcd_selec)
