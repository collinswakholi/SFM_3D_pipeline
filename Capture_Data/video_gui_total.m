function varargout = video_gui_total(varargin)
% VIDEO_GUI_TOTAL MATLAB code for video_gui_total.fig
%      VIDEO_GUI_TOTAL, by itself, creates a new VIDEO_GUI_TOTAL or raises the existing
%      singleton*.
%
%      H = VIDEO_GUI_TOTAL returns the handle to a new VIDEO_GUI_TOTAL or the handle to
%      the existing singleton*.
%
%      VIDEO_GUI_TOTAL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VIDEO_GUI_TOTAL.M with the given input arguments.
%
%      VIDEO_GUI_TOTAL('Property','Value',...) creates a new VIDEO_GUI_TOTAL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before video_gui_total_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to video_gui_total_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help video_gui_total

% Last Modified by GUIDE v2.5 09-Feb-2017 20:25:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @video_gui_total_OpeningFcn, ...
                   'gui_OutputFcn',  @video_gui_total_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end




% End initialization code - DO NOT EDIT


% --- Executes just before video_gui_total is made visible.
function video_gui_total_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to video_gui_total (see VARARGIN)

% Choose default command line output for video_gui_total
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
global Stop_no Stop_NIR_no Stop_Thermal_no Acquire Display helper
Stop_no=0;
Stop_NIR_no=0;
Stop_Thermal_no=0;
% NET.addAssembly('System');
Acquire=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.dll');
Display=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.display.dll');
helper=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.examples.helper.dll');

addpath('C:\Users\Owner\Desktop\New_gui_64');

set(handles.pushbutton18 ,'Enable' , 'off');
set(handles.pushbutton19 ,'Enable' , 'off');
set(handles.pushbutton21 ,'Enable' , 'off');
set(handles.pushbutton24 ,'Enable' , 'off');
set(handles.pushbutton25 ,'Enable' , 'off');
set(handles.pushbutton26 ,'Enable' , 'off');



% UIWAIT makes video_gui_total wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = video_gui_total_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fi
global video_start Stop_NIR_no

timeout_ms =500;
i=1;

video_start=logical(1);

pRequest='null';
lastRequestNr = -1;

while video_start

%     pause(0.1) 
    fi.imageRequestSingle();
    requestNr =fi.imageRequestWaitFor(timeout_ms);
    pRequest = fi.getRequest(requestNr);
    data = pRequest.bitmapData;
    bmp = data.bitmap;% pRequest.bitmapData.bitmap 경로 추출
    w = pRequest.imageWidth.read(); % 
    h = pRequest.imageHeight.read();
    
% lock bitmap into memory for reading
    bmpData = bmp.LockBits(System.Drawing.Rectangle(0, 0, w, h), ...
    System.Drawing.Imaging.ImageLockMode.ReadOnly, bmp.PixelFormat);
    % pRequest.bitmapData.bitmap 경로의 methods에서 LockBits를 호출함.(methods, Events, Superclasses-dispossable있음)
% get pointer to pixels, and copy RGB values into an array of bytes
    num = abs(bmpData.Stride) * h;
    bytes = NET.createArray('System.Byte', num);
    System.Runtime.InteropServices.Marshal.Copy(bmpData.Scan0, bytes, 0, num);    
    bmp.UnlockBits(bmpData);% unlock bitmap

% convert to MATLAB image
    bytes = uint8(bytes);
    img = reshape(bytes, [w,h])';
%     figure(1)
%     imshow(img)

    axes(handles.axes1); %클릭 이벤트를 통하여 axes2를 호출한다.
    imshow(img)
    drawnow
    
    % cleanup
    data.Dispose()
    bmp.Dispose()
    clear bmp data w h bmpData num bytes %img
        
   if (fi.isRequestNrValid(lastRequestNr)==1)
    % this image has been displayed thus the buffer is no longer needed...
       fi.imageRequestUnlock(lastRequestNr);
   end
    lastRequestNr = requestNr; 
%     top_state = get(handles.pushbutton2,);
%     if top_state
%         video_start=0;
%     end

    set(handles.pushbutton1,'BackgroundColor','Green')
end
%% 영상이 정지되면 자동 저장
% Stop_no=0;
axes(handles.axes2); 
imshow(img)
Date=datestr(now,'yy_mm_dd_HH_MM_SS');
name2=strcat('NIR_',Date,'.bmp');
imwrite(img,name2)
fprintf('NIR_save_no : %d, Name: %s \n' ,Stop_NIR_no,name2)
set(handles.pushbutton1,'BackgroundColor','red')


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global video_start Stop_NIR_no
video_start=0;
Stop_NIR_no=Stop_NIR_no+1;





% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fi

set(handles.pushbutton3,'BackgroundColor','red')

% NET.addAssembly('System');
% Acquire=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.dll');
% Display=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.display.dll');
% helper=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.examples.helper.dll');

% E = esig('mv.impact.acquire');
% helper=NET.addAssembly('F:\2016\call_C\bin\.NET\mv.impact.acquire.examples.helper.dll');
if (mv.impact.acquire.DeviceManager.deviceCount == 0)
    fprintf('No device found! Unable to continue! Press any key to end the program.\n');
    %input(a)
else
    mv.impact.acquire.LibraryPath.init(); 
    devCount = mv.impact.acquire.DeviceManager.deviceCount;
% show all devices
        for i=0:(devCount-1)
            pDev = mv.impact.acquire.DeviceManager.getDevice(i); %% 0 ~ N 번째 카메라 선택
            fprintf('Dev %d: %s \n', i,char(pDev.serial.read()));
        end
    % this will add the folders containing unmanaged libraries to the PATH variable.
pDev = mv.impact.acquire.DeviceManager.getDevice(1); % get the first device found%% 0 ~ N 번째 카메라 선택
fprintf('Initialising device %s. This might take some time...\n', char(pDev.serial.read()));
    % pDev.open();
 
    fi= mv.impact.acquire.FunctionInterface(pDev);
    
set(handles.pushbutton3,'BackgroundColor','green')
end



% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fi3
global video_start_Thermal Stop_Thermal_no

timeout_ms =500;
i=1;

video_start_Thermal=logical(1);

pRequest='null';
lastRequestNr = -1;

while video_start_Thermal

%     pause(0.1) 
    fi3.imageRequestSingle();
    requestNr =fi3.imageRequestWaitFor(timeout_ms);
    pRequest = fi3.getRequest(requestNr);
    data = pRequest.bitmapData;
    bmp = data.bitmap;% pRequest.bitmapData.bitmap 경로 추출
    w = pRequest.imageWidth.read(); % 
    h = pRequest.imageHeight.read();
    
% lock bitmap into memory for reading
    bmpData = bmp.LockBits(System.Drawing.Rectangle(0, 0, w, h), ...
    System.Drawing.Imaging.ImageLockMode.ReadOnly, bmp.PixelFormat);
    % pRequest.bitmapData.bitmap 경로의 methods에서 LockBits를 호출함.(methods, Events, Superclasses-dispossable있음)
% get pointer to pixels, and copy RGB values into an array of bytes
    num = abs(bmpData.Stride) * h;
    bytes = NET.createArray('System.Byte', num);
    System.Runtime.InteropServices.Marshal.Copy(bmpData.Scan0, bytes, 0, num);    
    bmp.UnlockBits(bmpData);% unlock bitmap

% convert to MATLAB image
    bytes = uint8(bytes);
    bytes2 =bytes(1:2:end);
    bytes3 =bytes(2:2:end);
    img = reshape(bytes2, [w,h])';
    img2 = reshape(bytes3, [w,h])';
    ref_no=190/256;
    ref=(double(img2)./ref_no);
    tem_img=ref+(double(img)/256);
    deg_img=tem_img-49;

    axes(handles.axes5); %클릭 이벤트를 통하여 axes2를 호출한다.
    imshow(deg_img,[10 25])    
    drawnow
    
  % cleanup
    data.Dispose()
    bmp.Dispose()
    clear bmp data w h bmpData num bytes img ref_no ref tem_img %deg_img
        
   if (fi3.isRequestNrValid(lastRequestNr)==1)
    % this image has been displayed thus the buffer is no longer needed...
       fi3.imageRequestUnlock(lastRequestNr);
   end
    lastRequestNr = requestNr; 
%     top_state = get(handles.pushbutton2,);
%     if top_state
%         video_start=0;
%     end
set(handles.pushbutton9,'BackgroundColor','green')
end
%% 영상이 정지되면 자동 저장
% Stop_no=0;
axes(handles.axes6); 
imshow(deg_img,[10 25])
% figure(1),imshow(deg_img,[])
%     colorbar('jet')
%     colorbar('off')
% name3=strcat('test_THERMAL_',num2str(Stop_Thermal_no),'.bmp');
% imwrite(deg_img,name3)
% fprintf('THERMAL_save_no : %d \n',Stop_Thermal_no)
Date=datestr(now,'yy_mm_dd_HH_MM_SS');
name=strcat('THERMAL_',Date,'.bmp');
imwrite(deg_img,name)
fprintf('THERMAL_save_no : %d, Name: %s \n' ,Stop_Thermal_no,name)
set(handles.pushbutton9,'BackgroundColor','red')


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global video_start_Thermal Stop_Thermal_no
video_start_Thermal=0;
Stop_Thermal_no=Stop_Thermal_no+1;


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fi3

set(handles.pushbutton11,'BackgroundColor','red')

% NET.addAssembly('System');
% Acquire=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.dll');
% Display=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.display.dll');
% helper=NET.addAssembly('C:\Users\Owner\Desktop\.NET\mv.impact.acquire.examples.helper.dll');

% E = esig('mv.impact.acquire');
% helper=NET.addAssembly('F:\2016\call_C\bin\.NET\mv.impact.acquire.examples.helper.dll');
if (mv.impact.acquire.DeviceManager.deviceCount == 0)
    fprintf('No device found! Unable to continue! Press any key to end the program.\n');
    %input(a)
else
    mv.impact.acquire.LibraryPath.init(); 
    devCount = mv.impact.acquire.DeviceManager.deviceCount;
% show all devices
        for i=0:(devCount-1)
            pDev3 = mv.impact.acquire.DeviceManager.getDevice(i); %% 0 ~ N 번째 카메라 선택
            fprintf('Dev %d: %s \n', i,char(pDev3.serial.read()));
        end
    % this will add the folders containing unmanaged libraries to the PATH variable.
pDev3 = mv.impact.acquire.DeviceManager.getDevice(0); % get the first device found%% 0 ~ N 번째 카메라 선택
fprintf('Initialising device %s. This might take some time...\n', char(pDev3.serial.read()));
    % pDev.open();
 
    fi3= mv.impact.acquire.FunctionInterface(pDev3);
end
set(handles.pushbutton11,'BackgroundColor','green')


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vid;
global hImage;

% 
% % vid = videoinput('gige', 1);
vid = videoinput('gige', 2, 'BayerRG8');

vid.ROIPosition = [0 0 1700 1712];

hImage=image(zeros(2712, 3384, 3), 'Parent', handles.axes3);

preview(vid, hImage);

set(handles.pushbutton17 ,'Enable' , 'off');
set(handles.pushbutton18 ,'Enable' , 'on');
set(handles.pushbutton19 ,'Enable' , 'on');
set(handles.pushbutton21 ,'Enable' , 'off');

    set(handles.pushbutton19, 'BackgroundColor', [0.598 0.797 0.996]);
    set(handles.pushbutton21, 'BackgroundColor', [0.996 0.598 0.398]);


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid

stoppreview(vid);
delete(vid);

set(handles.pushbutton17 ,'Enable' , 'on');
set(handles.pushbutton18 ,'Enable' , 'off');
set(handles.pushbutton19 ,'Enable' , 'off');
set(handles.pushbutton21 ,'Enable' , 'off');


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global  Acqui_stop vid name

Velocity_auto=12222;
PortN=3;
SlaveN=1;

set(handles.pushbutton19 ,'Enable' , 'off');
set(handles.pushbutton21 ,'Enable' , 'on');

Acqui_stop=0;
start(vid);

capture1=getsnapshot(vid);

Date=datestr(now,'yy_mm_dd_HH_MM_SS');
name=strcat('RGB_',Date,'_0.jpg');

% filename=strcat('IM',name,'_0.jpg');
imwrite(capture1, char(name));
pause(1) 


axes(handles.axes4);
imshow(capture1);

pause(1)

h = waitbar(0,'Please wait...', 'name', 'Status Bar');

for i=1:90
    
    if Acqui_stop==1;
        break
    end
    
    calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(12222), int32(Velocity_auto));
    
    pause(1.5)

    capture1=getsnapshot(vid);

    Date=datestr(now,'yy_mm_dd_HH_MM_SS');
    name=strcat('RGB_',Date,'_', num2str(i) ,'.jpg');
%     filename=strcat('IM',name,'_', num2str(i) ,'.jpg');
    imwrite(capture1, char(name));
    
    waitbar(i/90,h);
    
% 	axes(handles.axes4);
    imshow(capture1);
        
    
    
end

close(h);

if Acqui_stop==0

    calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
    , int32(0), int32(100000));

end




% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Acqui_stop

calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(3), int32(1));

msgbox('Emergency Stop!','Stop');

Acqui_stop=1;

set(handles.pushbutton19 ,'Enable' , 'on');
set(handles.pushbutton21 ,'Enable' , 'off');


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PortN=3;
Baudrate=115200;
SlaveN=1;
ServoS=1;


a=libfunctions('EziMOTIONPlusRx64');

if isempty(a)==0;
    fprintf('Exist Library \n');
else
    loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 
    fprintf('Load Complete!\n');
end

pause(2)
calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(PortN), int32(Baudrate));
fprintf('Connect Motor! \n');

pause(2)
calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(PortN), int32(SlaveN), int32(ServoS));
fprintf('Servo On! \n');


set(handles.pushbutton26 ,'Enable' , 'on');

% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global obj1

obj1 = instrfind('Type', 'serial', 'Port', 'COM4', 'Tag', '');

% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = serial('COM4');
else
    fclose(obj1);
    obj1 = obj1(1);
end

% Connect to instrument object, obj1.
fopen(obj1);

set(handles.pushbutton24 ,'Enable' , 'on');
set(handles.pushbutton25 ,'Enable' , 'on');
set(handles.pushbutton23 ,'Enable' , 'off');



% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global obj1

fclose(obj1);

set(handles.pushbutton24 ,'Enable' , 'off');
set(handles.pushbutton25 ,'Enable' , 'off');
set(handles.pushbutton23 ,'Enable' , 'on');


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2

global a

switch get(hObject,'Value')
    case 1
        a='2,WR,1,3,13';
    case 2
        a='2,WR,2,3,13';
    case 3
        a='2,WR,3,3,13';
    case 4
        a='2,WR,4,3,13';
    case 5
        a='2,WR,5,3,13';
    case 6
        a='2,WR,6,3,13';
    case 7
        a='2,WR,7,3,13';
    case 8
        a='2,WR,8,3,13';
    case 9
        a='2,WR,9,3,13';
    case 10
        a='2,WR,0,3,13';
    case 11
        a='2,WR,A,3,13';
    case 12
        a='2,WR,B,3,13';
    case 13
        a='2,WR,C,3,13';
    case 14
        a='2,WR,D,3,13';
    case 15
        a='2,WR,E,3,13';
    case 16
        a='2,WR,F,3,13';
    case 17
        a='2,WR,G,3,13';
    case 18
        a='2,WR,H,3,13';
end


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global a obj1

fprintf(obj1, a);


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(3), int32(1)...
    , int32(0), int32(100000));
