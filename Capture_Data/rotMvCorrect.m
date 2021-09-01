function Im = rotMvCorrect(I, Ang, gamma, T)

% function perfoms 90-degree image rotations, gamma and color correction of input data
I1 = I;
s1 = size(I);

assert(length(s1)==3,'Please use a color image...')
assert(Ang<4, 'Please enter correct value of rotation angle (0,1,2,3)')
assert((gamma>0)&&(gamma<3),'Gamma values should be between 0 and 3')

II = mat2gray(rot90((imadjust(I1,[],[],gamma)),Ang));

s1 = size(II);
    Rlayer = II(:,:,1);
    Glayer = II(:,:,2);
    Blayer = II(:,:,3);
    
    Im = uint8(255*reshape((T*reshape(II,[s1(1)*s1(2) 3])')',[s1(1) s1(2) 3]));
    
