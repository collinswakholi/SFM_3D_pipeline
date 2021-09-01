function Im = mvColorCorrect(I, T)
% I is the image to be corrected
% T is the color twist matrix from matrix vision

% Im is the output image

assert(nargin == 2, 'Please check your inputs')

s1 = size(I);
assert(length(s1) == 3, 'Operation for Color (MxNx3) images only')

I2 = mat2gray(I);
    Rlayer = I2(:,:,1);
    Glayer = I2(:,:,2);
    Blayer = I2(:,:,3);
    
    Im = uint8(255*reshape((T*reshape(I2,[s1(1)*s1(2) 3])')',[s1(1) s1(2) 3]));
    
    