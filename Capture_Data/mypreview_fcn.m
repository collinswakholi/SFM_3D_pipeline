function mypreview_fcn(obj, event, himage)
global Ang;
Img = rot90(event.Data,Ang);
set(himage, 'cdata', Img);