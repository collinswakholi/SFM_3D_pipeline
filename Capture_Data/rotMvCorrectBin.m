function Im = rotMvCorrectBin(I, Ang, gamma, T, binning)

% function perfoms 90-degree image rotations, gamma and color correction of
% input Image

I = imresize(I,(1/binning));

assert(Ang<4, 'Please enter correct value of rotation angle (0,1,2,3)')
assert((gamma>0)&&(gamma<3),'Gamma values should be between 0 and 3')

I = mat2gray(rot90((imadjust(I,[],[],gamma)),Ang));

s1 = size(I);

Im = uint8(255*reshape((T*reshape(I,[s1(1)*s1(2) 3])')',[s1(1) s1(2) 3]));
    
