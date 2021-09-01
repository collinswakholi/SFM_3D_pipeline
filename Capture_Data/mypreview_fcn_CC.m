function mypreview_fcn_CC(obj, event, himage)

global T

Img = mvColorCorrect(event.Data,T);
set(himage, 'cdata', Img);