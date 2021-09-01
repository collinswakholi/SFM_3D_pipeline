function mypreview_fcn_rotCC(obj, event, himage)

global Ang S_gamma T

Img = rotMvCorrect(event.Data,Ang, S_gamma, T);
set(himage, 'cdata', Img);