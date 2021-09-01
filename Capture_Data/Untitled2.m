% check property of src
vid = videoinput('gentl', 1, 'BayerRG8');
src = getselectedsource(vid);
% aaa = propinfo(src,'ColorTransformationValueSelector');
I = getsnapshot(vid);
I2 = mat2gray(imadjust(I,[],[],0.5));
    Rlayer = I2(:,:,1);
    Glayer = I2(:,:,2);
    Blayer = I2(:,:,3);

T = [1.79386, -0.64138, -0.12276;...
        -.37305, 1.67433,-0.27112;...
        0.10253,-0.72658,1.65402];% from matrix vision

s1 = size(I);

Ixyz = reshape((T*reshape(I2,[s1(1)*s1(2) 3])')',[s1(1) s1(2) 3]);

imshowpair(I,Ixyz,'montage');