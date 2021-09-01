%////////////////////////////////
global cameraObj ;
global horizontal_flip;
horizontal_flip = 0;
global vertical_flip;
vertical_flip = 0;
global rotation;
rotation = 0;
cameraObj = videoinput('winvideo',selection); %selection is the camera device ID (in my case works for microscope cameras as well as webcams and anything connected through the manyCam software)
axes(handles.axesCamera); %sets the focus on the desired axis of the GUI
vidRes = cameraObj.VideoResolution;
nBands = cameraObj.NumberOfBands;
hImage = image( zeros(vidRes(2), vidRes(1), nBands) );
axis off;
PreviewHandle = preview(cameraObj,hImage); % the handle of the preview object
% //////////////////////////  then in specific callback for radiobutton or pushbutton here is the code
% --- Executes on button press in radiobuttonHorizontalFlip.
function radiobuttonHorizontalFlip_Callback(hObject, eventdata, handles)
global PreviewHandle;
global horizontal_flip;
if get(hObject,'Value')
      horizontal_flip = 1;
else
      horizontal_flip = 0;
end
setappdata(PreviewHandle,'UpdatePreviewWindowFcn',@mypreview_fcn);

% --- Executes on button press in radiobuttonVerticalFlip.
function radiobuttonVerticalFlip_Callback(hObject, eventdata, handles)
global PreviewHandle;
global vertical_flip;
if get(hObject,'Value')
      vertical_flip = 1;
else
      vertical_flip = 0;
end
setappdata(PreviewHandle,'UpdatePreviewWindowFcn',@mypreview_fcn);

% --- Executes on button press in pushbuttonRotatePlus90deg.
function pushbuttonRotatePlus90deg_Callback(hObject, eventdata, handles)
global PreviewHandle;
global rotation;
rotation = rotation + 1;
setappdata(PreviewHandle,'UpdatePreviewWindowFcn',@mypreview_fcn);

% --- Executes on button press in pushbuttonRotateMinus90deg.
function pushbuttonRotateMinus90deg_Callback(hObject, eventdata, handles)
global PreviewHandle;
global rotation;
rotation =  rotation - 1;
setappdata(PreviewHandle,'UpdatePreviewWindowFcn',@mypreview_fcn);

function mypreview_fcn(obj, event, himage)
global rotation;
Img = rot90(event.Data,rotation);
set(himage, 'cdata', Img);
%////////////////////