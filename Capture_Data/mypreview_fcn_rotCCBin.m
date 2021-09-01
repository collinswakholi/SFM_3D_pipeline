function mypreview_fcn_rotCCBin(obj, event, himage)

global Ang S_gamma T binn

Img = rotMvCorrectBin(event.Data,Ang, S_gamma, T, binn);
set(himage, 'cdata', Img);