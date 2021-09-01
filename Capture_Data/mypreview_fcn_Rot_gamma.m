function mypreview_fcn_Rot_gamma(obj, event, himage)

% global Ang S_gamma
sg = Sgamma;
ang1 = Ang;

Img = rot90(imadjust(event.Data,[],[],sg),ang1);
set(himage, 'cdata', Img);