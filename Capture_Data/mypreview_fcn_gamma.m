function mypreview_fcn_Rot_gamma(obj, event, himage)
global 
global S_gamma;
Img = imadjust(event.Data,[],[],S_gamma);
set(himage, 'cdata', Img);