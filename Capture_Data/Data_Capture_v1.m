function varargout = Data_Capture_v1(varargin)
% DATA_CAPTURE_V1 MATLAB code for Data_Capture_v1.fig
%      DATA_CAPTURE_V1, by itself, creates a new DATA_CAPTURE_V1 or raises the existing
%      singleton*.
%
%      H = DATA_CAPTURE_V1 returns the handle to a new DATA_CAPTURE_V1 or the handle to
%      the existing singleton*.
%
%      DATA_CAPTURE_V1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DATA_CAPTURE_V1.M with the given input arguments.
%
%      DATA_CAPTURE_V1('Property','Value',...) creates a new DATA_CAPTURE_V1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Data_Capture_v1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Data_Capture_v1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Data_Capture_v1

% Last Modified by GUIDE v2.5 18-Jun-2019 17:29:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Data_Capture_v1_OpeningFcn, ...
                   'gui_OutputFcn',  @Data_Capture_v1_OutputFcn, ...
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


% --- Executes just before Data_Capture_v1 is made visible.
function Data_Capture_v1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Data_Capture_v1 (see VARARGIN)

% Choose default command line output for Data_Capture_v1
handles.output = hObject;

global com_ports motor_speed duration_time direction no_feed please_wait
global Ang S_gamma T ECC
global T_old Ang_old S_gamma_old
global ss binn vid_1 vid_2 vid_3
global stop_position I1 I2 n_images3
global selected_dir save_dir3 count
global select_cam1 select_cam2 select_cam3

instrreset;
imaqreset;

count = 1;
motor_speed = 4800;
stop_position = 990000;

set(handles.motorSpeed,'String',motor_speed)
set(handles.motorOnDuration,'String',stop_position)

a=libfunctions('EziMOTIONPlusRx64');

if length(a)>1
    unloadlibrary('EziMOTIONPlusRx64'); 
    loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 
else
%  a=libfunctions('EziMOTIONPlusRx64');
 
%  if length(a)<1
    loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 
 end
% a=libfunctions('EziMOTIONPlusRx64');

% read and display COM ports available
com_names = seriallist;
com_ports = strtok(com_names,'COM');
set(handles.comPorts,'String',com_ports)

% default values


motor_speed = 0;
duration_time = 100000;
direction = 1; %1 for clockwise and -1 for anticlockwise
[Ang,Ang_old] = deal(0);
[S_gamma_old,S_gamma] = deal(0.4545);
set(handles.gamma, 'String',S_gamma);

T = ([1, 0, 0; 0, 1, 0; 0, 0, 1]);% no correction
T_old = ([1, 0, 0; 0, 1, 0; 0, 0, 1]);
ss = 0;
binn = 4;
ECC = 1;

if ECC == 1
    T = [1.79386, -0.64138, -0.12276;...
        -.37305, 1.67433,-0.27112;...
        0.10253,-0.72658,1.65402];% from matrix vision
else
    T = T;
end

set(handles.colorCorrectionCB, 'value', ECC)

[select_cam1, select_cam2, select_cam3] = deal(0);

no_feed = imread('no_feed.jpg');
please_wait = imread('Please_Wait.png');
[I1, I2] = deal(zeros(2464, 2056, 3));

vid_1 = [];
vid_2 = [];
vid_3 = [];

selected_dir = '';
save_dir3 = '';
n_images3 = 36;
set(handles.saveData, 'Enable', 'off');
set(handles.Cap_new, 'Enable', 'off');


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Data_Capture_v1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Data_Capture_v1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in load_motor.
function load_motor_Callback(hObject, eventdata, handles)
% hObject    handle to load_motor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global m_slave select_port Baud_Rate Check

%open port with motor
calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(select_port), int32(Baud_Rate));

Check=calllib('EziMOTIONPlusRx64', 'FAS_IsSlaveExist',int32(select_port), int32(m_slave));


% check connection

if Check==1
    
    msgbox('Connected Successfully!','Success');
   
    
    set(handles.load_motor, 'BackgroundColor', [0.75 0.39 0.39]);
    
else
    msgbox('Cannot Connect \n Please check Slave...','Error','Error')
    
    set(handles.load_motor, 'BackgroundColor', [0.199 0.199 0.199]);
    
end


% --- Executes on selection change in comPorts.
function comPorts_Callback(hObject, eventdata, handles)
% hObject    handle to comPorts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global select_port

 port_string = get(hObject,'String');
 pos = get(hObject,'Value');
 
select_port = str2num(port_string{pos})
% disp(strcat('COM',select_port));


% --- Executes during object creation, after setting all properties.
function comPorts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to comPorts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in baud_rate.
function baud_rate_Callback(hObject, eventdata, handles)
% hObject    handle to baud_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Baud_Rate

switch get(hObject,'Value')

    case 1
        Baud_Rate=9600;        
    case 2
        Baud_Rate=38400;
    case 3
        Baud_Rate=57600;
    case 4
        Baud_Rate=115200;
    case 5
        Baud_Rate=128000;
end
% rrr  = Baud_Rate
% Hints: contents = cellstr(get(hObject,'String')) returns baud_rate contents as cell array
%        contents{get(hObject,'Value')} returns selected item from baud_rate

% disp(strcat('BaudRate = ',Baud_Rate));


% --- Executes during object creation, after setting all properties.
function baud_rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to baud_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in motor_slave.
function motor_slave_Callback(hObject, eventdata, handles)
% hObject    handle to motor_slave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global m_slave 

switch get(hObject,'Value')

    case 1
        m_slave=0;        
    case 2
        m_slave=1;
    case 3
        m_slave=2;
    case 4
        m_slave=3;
    case 5
        m_slave=4;
    case 6
        m_slave=5;
    case 7
        m_slave=6;
    case 8
        m_slave=7;
end
%  aaa =m_slave
% disp(strcat('Motor slave# = ',m_slave));

% Hints: contents = cellstr(get(hObject,'String')) returns motor_slave contents as cell array
%        contents{get(hObject,'Value')} returns selected item from motor_slave


% --- Executes during object creation, after setting all properties.
function motor_slave_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_slave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motorSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to motorSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global motor_speed

motor_speed = str2double(get(hObject,'String'));


% Hints: get(hObject,'String') returns contents of motorSpeed as text
%        str2double(get(hObject,'String')) returns contents of motorSpeed as a double


% --- Executes during object creation, after setting all properties.
function motorSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motorSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motorOnDuration_Callback(hObject, eventdata, handles)
% hObject    handle to motorOnDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global stop_position

stop_position = str2double(get(hObject,'String'))

% Hints: get(hObject,'String') returns contents of motorOnDuration as text
%        str2double(get(hObject,'String')) returns contents of motorOnDuration as a double


% --- Executes during object creation, after setting all properties.
function motorOnDuration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motorOnDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in startMotor.
function startMotor_Callback(hObject, eventdata, handles)
% hObject    handle to startMotor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global select_port m_slave  stop_position motor_speed direction load_servo load_motor
global n_images3 rev_time delay_time

ParamVal=lib.pointer('uint32Ptr',1);

Position_INC1=stop_position*direction;


pause(0.3)
    load_motor = calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(select_port), int32(m_slave),...
        int32(stop_position), int32(motor_speed));


if load_motor == 0
    
    msgbox('Motor Connected Successfully!','Success');
    set(handles.startMotor, 'BackgroundColor', [0 0.797 0.398]);
    
else
    msgbox('Cannot Connect motor... Please reset and try again','Error','Error')
    set(handles.startMotor, 'BackgroundColor', [0.199 0.199 0.199]);
end

if motor_speed > 1
%     rev_time = round((23959*motor_speed^(-1.022)),2); %T = 23959P^(-1.022) from stats
    rev_time = 20.5;
    delay_time = round((rev_time/n_images3),4);
    set(handles.revTime,'String',num2str(rev_time));
end


% --- Executes on button press in servoOn.
function servoOn_Callback(hObject, eventdata, handles)
% hObject    handle to servoOn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global ServoS select_port m_slave Check load_servo
servo_on = get(hObject, 'Value');

if Check == 1
    if servo_on == 1
        ServoS = 1;
        calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(select_port), int32(m_slave), int32(ServoS))
        set(handles.servoOn, 'BackgroundColor', [0 0.797 0.398]);
        disp('Servo Connected...')
    else
        ServoS = 0;
        calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(select_port), int32(m_slave), int32(ServoS))
        set(handles.servoOn, 'BackgroundColor', [0.94 0.94 0.94]);
        set(handles.startMotor, 'BackgroundColor', [0.94 0.94 0.94]);
        disp('Servo Disconnected...')
    end
else
        msgbox('Could NOT Connect... Please check values!','Error','Error')

        set(handles.servoOn, 'BackgroundColor', [0.199 0.199 0.199]);
end
  

% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global select_port m_slave load_motor

if load_motor == 0
    calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(select_port), int32(m_slave));
    msgbox('Motor Stopped!','Stop');
   
    set(handles.servoOn, 'BackgroundColor', [0.94 0.94 0.94]);
    set(handles.startMotor, 'BackgroundColor', [0.94 0.94 0.94]);
else
    msgbox('No motor loaded...','Stop');
end


% --- Executes on button press in loadCameras.
function loadCameras_Callback(hObject, eventdata, handles)
% hObject    handle to loadCameras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global select_cam1 select_cam2 select_cam3 vid_1 vid_2 vid_3
global src_1 src_2 src_3 fr
global exp new_exp name1 name2 name3
% global frame_rate_1 frame_rate_2 frame_rate_3

imaqreset;

if select_cam1 == 1
    vid_1 = videoinput('gentl',2,'BayerRG8');
    src_1 = getselectedsource(vid_1);
    vid_1.FramesPerTrigger = 1;
%     frame_rate_1 = src_1.AcquisitionFrameRate;
    exp = src_1.ExposureTime;
    fr1 = src_1.mvResultingFrameRate;
%     set(handles.exposure,'String',exp);
    name1 = vid_1.Name
    fprintf('%s\n Camera_1 Sucessfully loaded');
else
    fprintf('%s\n Camera_1 not selected');
    name1=[];
    fr1 = 0;
end

if select_cam2 == 1
    vid_2 = videoinput('gentl',1,'BayerRG8');
    src_2 = getselectedsource(vid_2);
    vid_2.FramesPerTrigger = 1;
%     frame_rate_2 = src_2.AcquisitionFrameRate;
    exp = src_2.ExposureTime;
    fr2 = src_2.mvResultingFrameRate;
%     set(handles.exposure,'String',exp);
    
    name2 = vid_2.Name
    fprintf('%s\n Camera_2 Sucessfully loaded');
else
    name2 = [];
    fprintf('%s\n Camera_2 not selected');
    fr2=0;
end

if select_cam3 == 1
    vid_3 = videoinput('gentl',3,'BayerRG8');
    src_3 = getselectedsource(vid_3);
    vid_3.FramesPerTrigger = 1;
%     frame_rate_3 = src_3.AcquisitionFrameRate;
    exp = src_3.ExposureTime;
%     set(handles.exposure,'String',exp);
    fr = src_3.mvResultingFrameRate;
    name3 = vid_3.Name
    fprintf('%s\n Camera_3 Sucessfully loaded');
else
    name3 = [];
    fprintf('%s\n Camera_3 not selected');
    fr=0;
end

set(handles.exposure,'String',exp);

txt1 = strcat('Camera 1 (',num2str(fr1),'fps)');
txt2 = strcat('Camera 2 (',num2str(fr2),'fps)');
txt3 = strcat('Camera 3 (',num2str(fr),'fps)');
% set(handle.cam1_txt, 'String', txt1)
% set(handle.cam2_txt, 'String', txt2)
% set(handle.cam3_txt, 'String', txt3)

new_exp = exp;
    set(handles.loadCameras,'BackgroundColor','green');
    



% --- Executes on button press in previewCams.
function previewCams_Callback(hObject, eventdata, handles)
% hObject    handle to previewCams (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid_1 vid_2 vid_3 src_1 src_2 src_3 hImage_1 hImage_2 hImage_3
global name1 name2 name3
global no_feed noFeed1 noFeed2 noFeed3 binn new_exp
global Ang S_gamma T


no_feed_gray = rgb2gray(no_feed);
sz1 = size(no_feed_gray);

[src_1.ExposureTime, src_2.ExposureTime, src_3.ExposureTime] = deal(new_exp);
% set(handles.exposure,'String',num2str(round(binn*binn*new_exp)));
set(handles.exposure,'String',num2str(new_exp));

% [src_1.BinningVertical, src_1.BinningHorizontal,...
%     src_2.BinningVertical, src_2.BinningHorizontal,...
%     src_3.BinningVertical, src_3.BinningHorizontal] = deal(binn);

Dim_12 = [2464/binn,2056/binn,3];
Dim_3 = [4096/binn,2176/binn,3];

if length(name1) > 1
    hImage_1=image(zeros(Dim_12), 'Parent', handles.axes_cam_1);
    setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
    preview(vid_1, hImage_1);
else
    hImage_1=image(zeros(Dim_12), 'Parent', handles.axes_cam_1);
    noFeed1 = padarray(no_feed_gray, round(0.5*(Dim_12(1:2)-sz1)));
    imshow(noFeed1,'Parent',handles.axes_cam_1)
end

if length(name2) > 1
    hImage_2=image(zeros(Dim_12), 'Parent', handles.axes_cam_2);
    setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
    preview(vid_2, hImage_2);
else
    hImage_2=image(zeros(Dim_12), 'Parent', handles.axes_cam_2);
    noFeed2 = padarray(no_feed_gray, round(0.5*(Dim_12(1:2)-sz1)));
    imshow(noFeed2,'Parent',handles.axes_cam_2)
end

if length(name3) > 1
    hImage_3=image(zeros(Dim_3), 'Parent', handles.axes_cam_3);
    setappdata(hImage_3,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
    preview(vid_3, hImage_3);
else
    hImage_3=image(zeros(Dim_3), 'Parent', handles.axes_cam_3);
    noFeed3 = padarray(no_feed_gray, round(0.5*(Dim_3(1:2)-sz1)));
    imshow(noFeed3,'Parent',handles.axes_cam_3)
end


% --- Executes on selection change in rotAngle.
function rotAngle_Callback(hObject, eventdata, handles)
% hObject    handle to rotAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Ang


    str = get(hObject, 'String');
    val = get(hObject,'Value');

    switch str{val}
        
    case '0' 
       handles.rotAngle = 1;
       Ang=0; 

    case '90' 
       handles.rotAngle = 2;
       Ang=1;

    case '180' 
       handles.rotAngle = 3;
       Ang=2;

    case '270' 
       handles.rotAngle = 4;
       Ang=3;
      end
   


% Hints: contents = cellstr(get(hObject,'String')) returns rotAngle contents as cell array
%        contents{get(hObject,'Value')} returns selected item from rotAngle


% --- Executes during object creation, after setting all properties.
function rotAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rotAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in rotAngle_2.
function rotAngle_2_Callback(hObject, eventdata, handles)
% hObject    handle to rotAngle_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns rotAngle_2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from rotAngle_2


% --- Executes during object creation, after setting all properties.
function rotAngle_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rotAngle_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in rotAngle_3.
function rotAngle_3_Callback(hObject, eventdata, handles)
% hObject    handle to rotAngle_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns rotAngle_3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from rotAngle_3


% --- Executes during object creation, after setting all properties.
function rotAngle_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rotAngle_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in exposure.
function exposure_Callback(hObject, eventdata, handles)
% hObject    handle to exposure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global new_exp vid_1 src_1 hImage_1 vid_2 src_2 hImage_2 vid_3 src_3 hImage_3
global S_gamma Ang T 
global name1 name2 name3 binn


new_exp = str2double(get(hObject,'String'));

src_1.ExposureTime = int32(new_exp);
src_2.ExposureTime = int32(new_exp);
src_3.ExposureTime = int32(new_exp);

       if length(name1)>1
            closepreview(vid_1)
%             setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_1, hImage_1);
            drawnow
       end
       
       if length(name2)>1
            closepreview(vid_2)
%             setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_2, hImage_2);
            drawnow
       end
       
      if length(name3)>1
            closepreview(vid_3)
%             setappdata(hImage_3,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_3,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_3, hImage_3);
            drawnow
       end


% Hints: contents = cellstr(get(hObject,'String')) returns exposure contents as cell array
%        contents{get(hObject,'Value')} returns selected item from exposure


% --- Executes during object creation, after setting all properties.
function exposure_CreateFcn(hObject, eventdata, handles)
% hObject    handle to exposure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in exposure_2.
function exposure_2_Callback(hObject, eventdata, handles)
% hObject    handle to exposure_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

       
% Hints: contents = cellstr(get(hObject,'String')) returns exposure_2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from exposure_2


% --- Executes during object creation, after setting all properties.
function exposure_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to exposure_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in exposure_3.
function exposure_3_Callback(hObject, eventdata, handles)
% hObject    handle to exposure_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     
% Hints: contents = cellstr(get(hObject,'String')) returns exposure_3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from exposure_3


% --- Executes during object creation, after setting all properties.
function exposure_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to exposure_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu15.
function popupmenu15_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu15 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu15


% --- Executes during object creation, after setting all properties.
function popupmenu15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu16.
function popupmenu16_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu16 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu16


% --- Executes during object creation, after setting all properties.
function popupmenu16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu17.
function popupmenu17_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu17 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu17


% --- Executes during object creation, after setting all properties.
function popupmenu17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in updateCamera.
function updateCamera_Callback(hObject, eventdata, handles)
% hObject    handle to updateCamera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in resetCamera.
function resetCamera_Callback(hObject, eventdata, handles)
% hObject    handle to resetCamera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global T Ang S_gamma new_exp T_old Ang_old S_gamma_old exp binn
global vid_1 hImage_1 vid_2 hImage_2 vid_3 hImage_3 src_1 src_2 src_3
global name1 name2 name3

T = T_old;
S_gamma = S_gamma_old;
Ang = Ang_old;
expo = int32(exp);

src_1.ExposureTime = expo;
src_2.ExposureTime = expo;
src_3.ExposureTime = expo;

       if length(name1)>1
            closepreview(vid_1)
%             setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_1, hImage_1);
            drawnow
       end
       
       if length(name2)>1
            closepreview(vid_2)
%             setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_2, hImage_2);
            drawnow
       end
       
      if length(name3)>1
            closepreview(vid_3)
%             setappdata(hImage_3,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_3,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_3, hImage_3);
            drawnow
       end

% set(handles.exposure,'String',num2str(exp*binn*binn));
set(handles.exposure,'String',num2str(exp));
set(handles.rotAngle,'Value',(1+Ang));
set(handles.gamma,'String',S_gamma);
set(handles.colorCorrectionCB,'Value',0)
set(handles.binningPop,'Value',4)




function gamma_Callback(hObject, eventdata, handles)
% hObject    handle to gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global S_gamma 

    S_gamma = str2double(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of gamma as text
%        str2double(get(hObject,'String')) returns contents of gamma as a double


% --- Executes during object creation, after setting all properties.
function gamma_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function gamma_2_Callback(hObject, eventdata, handles)
% hObject    handle to gamma_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gamma_2 as text
%        str2double(get(hObject,'String')) returns contents of gamma_2 as a double


% --- Executes during object creation, after setting all properties.
function gamma_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gamma_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function gamma_3_Callback(hObject, eventdata, handles)
% hObject    handle to gamma_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gamma_3 as text
%        str2double(get(hObject,'String')) returns contents of gamma_3 as a double


% --- Executes during object creation, after setting all properties.
function gamma_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gamma_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in colorCorrectionCB.
function colorCorrectionCB_Callback(hObject, eventdata, handles)
% hObject    handle to colorCorrectionCB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global T ECC

ECC = get(hObject,'Value');

if ECC == 1
    T = [1.79386, -0.64138, -0.12276;...
        -.37305, 1.67433,-0.27112;...
        0.10253,-0.72658,1.65402];% from matrix vision
else
    T = [1,0,0;0,1,0;0,0,1];
end
% Hint: get(hObject,'Value') returns toggle state of colorCorrectionCB


% --- Executes on button press in capture_3.
function capture_3_Callback(hObject, eventdata, handles)
% hObject    handle to capture_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vid_3 src_3 name3 hImage_3 new_exp vid_1 vid_2
% global vid_3 src_3 name3 hImage_3 new_exp
global T Ang S_gamma binn rev_time n_images3 
global I1 I2 please_wait Images fr

closepreview(vid_1)
closepreview(vid_2)
closepreview(vid_3)

please_wait_gray = rgb2gray(please_wait);
Dim_3 = [4096,2176,3];
sz1 = size(please_wait_gray);
pleaseWait= padarray(please_wait_gray, round(0.5*(Dim_3(1:2)-sz1)));

if (length(size(I1))<2)&&(select_cam1==1)&&(select_cam2==1)
    I1 = getsnapshot(vid_1);
    I1 = rotMvCorrect(I1,Ang,S_gamma,T);
    
    I2 = getsnapshot(vid_2);
    I2 = rotMvCorrect(I2,Ang,S_gamma,T);
end

imshow(I1,[],'Parent',handles.axes_cam_1)
imshow(I2,[],'Parent',handles.axes_cam_2)
imshow(pleaseWait,[],'Parent',handles.axes_cam_3)
drawnow

% images = zeros(2176,4096,3,n_images3);
Images = {};

% [src_3.BinningVertical, src_3.BinningHorizontal] = deal(1);

% Exp = int32((binn*binn)*new_exp);
Exp = double(new_exp);

src_3.ExposureTime = Exp;
% vid_3.FramesPerTrigger = 1;

% src_3.mvResultingFrameRate=15;
src_3.AcquisitionFrameRate=15;

nframe_rate = 10;
fr = src_3.mvResultingFrameRate;
if fr<10
    errordlg('Camera Framerate of  is too low')
else
        if fr > nframe_rate
            nframe_rate = 0.8*fr;
        end
        disp(strcat('Capture fps = ',num2str(nframe_rate)))
        src_3.AcquisitionFrameRate = nframe_rate;

        total_frames = (nframe_rate*rev_time);

        vid_3.LoggingMode = 'memory';
        triggerconfig(vid_3, 'immediate')
        vid_3.TimerPeriod = rev_time; 

        vid_3.FramesPerTrigger = n_images3; 
        vid_3.FrameGrabInterval = (total_frames/n_images3);

        disp('+ [Capturing images from Cam 3]...')
        % Initiate the acquisition.
        start(vid_3);tic

        % Wait for acquisition to end.
        wait(vid_3, (1.2*rev_time));toc

        % Determine the number frames acquired.
        framesCaptured = vid_3.FramesAcquired;


        [images time] =getdata(vid_3);

        disp(strcat(num2str(framesCaptured),'_Frames have been captured from Cam 3]...'))

        disp('+ [Correcting color of images]...')
        images = gpuArray(images);
        for i = 1:n_images3
            Images{i} = gather(rotMvCorrect(images(:,:,:,i),Ang,S_gamma,T));
        end
        imshow(Images{1},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String','1');
        drawnow
        disp('+ Done...')
        set(handles.saveData, 'Enable', 'on');
        set(handles.Cap_new, 'Enable', 'on');
end



% --- Executes on button press in closeReset.
function closeReset_Callback(hObject, eventdata, handles)
% hObject    handle to closeReset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid_1 vid_2 vid_3 select_port m_slave load_motor

answer = questdlg('Are you sure you want to Exit now?', ...
	'Warning!!!', ...
	'Yes','No thank you','No thank you');

if answer == 'Yes'
    if load_motor == 0
        calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(select_port), int32(m_slave));
        calllib('EziMOTIONPlusRx64', 'FAS_ServoAlarmReset', int32(select_port), int32(m_slave));
        ServoS = 1;
        calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(select_port), int32(m_slave), int32(ServoS));
    end

    closepreview(vid_1)
    closepreview(vid_2)
    closepreview(vid_3)

    cla
    instrreset
    imaqreset
    close all
else
    okay=1
end

% --- Executes on button press in saveData.
function saveData_Callback(hObject, eventdata, handles)
% hObject    handle to saveData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Images n_images3 save_dir3


   save_dir3 = uigetdir('F:\Image data');

    
   disp('+ [Saving images]...')
   
%    img = zeros(4096,2176,3);
    
    for i = 1:n_images3 
        baseFileName = (strcat(num2str(i),'.tif'));
        fullFileName = fullfile(save_dir3, baseFileName);
        imwrite(Images{i}, fullFileName);
        disp(strcat('Saving image_',num2str(i),'_out of_', num2str(n_images3)))
    end

disp('+ [Done]...')
set(handles.saveData, 'Enable', 'off');

% --------------------------------------------------------------------
function file_tag_Callback(hObject, eventdata, handles)
% hObject    handle to file_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function calibration_tag_Callback(hObject, eventdata, handles)
% hObject    handle to calibration_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function resetEnv_Callback(hObject, eventdata, handles)
% hObject    handle to resetEnv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function exit_program_Callback(hObject, eventdata, handles)
% hObject    handle to exit_program (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function new_calibration_Callback(hObject, eventdata, handles)
% hObject    handle to new_calibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function cal_cam_1_Callback(hObject, eventdata, handles)
% hObject    handle to cal_cam_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function cal_cam_2_Callback(hObject, eventdata, handles)
% hObject    handle to cal_cam_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function cal_cam_3_Callback(hObject, eventdata, handles)
% hObject    handle to cal_cam_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Alarm_reset.
function Alarm_reset_Callback(hObject, eventdata, handles)
% hObject    handle to Alarm_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global select_port m_slave 

    calllib('EziMOTIONPlusRx64', 'FAS_ServoAlarmReset', int32(select_port), int32(m_slave));
    
    set(handles.startMotor, 'BackgroundColor', [0.94 0.94 0.94]);
        msgbox('Reset Successfull...','Success');
 


% --- Executes on button press in selectCam1.
function selectCam1_Callback(hObject, eventdata, handles)
% hObject    handle to selectCam1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of selectCam1
global select_cam1

select_cam1 = get(hObject,'Value')

% --- Executes on button press in selectCam2.
function selectCam2_Callback(hObject, eventdata, handles)
% hObject    handle to selectCam2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of selectCam2
global select_cam2

select_cam2 = get(hObject,'Value')

% --- Executes on button press in selectCam3.
function selectCam3_Callback(hObject, eventdata, handles)
% hObject    handle to selectCam3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of selectCam3
global select_cam3

select_cam3 = get(hObject,'Value')


function imageNumber_Callback(hObject, eventdata, handles)
% hObject    handle to imageNumber (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of imageNumber as text
%        str2double(get(hObject,'String')) returns contents of imageNumber as a double


% --- Executes during object creation, after setting all properties.
function imageNumber_CreateFcn(hObject, eventdata, handles)
% hObject    handle to imageNumber (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in previousImage.
function previousImage_Callback(hObject, eventdata, handles)
% hObject    handle to previousImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global count Images n_images3

    ii = count-1;
    if (ii > 0) && ii <= (n_images3)
        imshow(Images{ii},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String',num2str(ii));
    elseif ii < 1
        imshow(Images{1},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String','1');
    elseif ii>n_images3
        imshow(Images{n_images3},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String',num2str(n_images3));
    end
    count = ii;



% --- Executes on button press in nextImage.
function nextImage_Callback(hObject, eventdata, handles)
% hObject    handle to nextImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global count Images n_images3

    ii = count+1;
    if (ii > 0) && ii <= (n_images3)
        imshow(Images{ii},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String',num2str(ii));
    elseif ii < 1
        imshow(Images{1},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String','1');
    elseif ii>n_images3
        imshow(Images{n_images3},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String',num2str(n_images3));
    end
    count = ii;

function nImage3_Callback(hObject, eventdata, handles)
% hObject    handle to nImage3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global n_images3 rev_time delay_time motor_speed

n_images3 = str2double(get(hObject,'String'))


% Hints: get(hObject,'String') returns contents of nImage3 as text
%        str2double(get(hObject,'String')) returns contents of nImage3 as a double


% --- Executes during object creation, after setting all properties.
function nImage3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nImage3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function nImage_12_Callback(hObject, eventdata, handles)
% hObject    handle to nImage_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nImage_12 as text
%        str2double(get(hObject,'String')) returns contents of nImage_12 as a double


% --- Executes during object creation, after setting all properties.
function nImage_12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nImage_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in capture_12.
function capture_12_Callback(hObject, eventdata, handles)
% hObject    handle to capture_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vid_1 vid_2 src_1 src_2 hImage_1 hImage_2
global name1  name2  
global T Ang S_gamma no_feed binn
global ss new_exp selected_dir I1 I2

no_feed_gray = rgb2gray(no_feed);
sz1 = size(no_feed_gray);

if length(name1) > 1
    closepreview(vid_1)
end

if length(name2) > 1
    closepreview(vid_2)
end

Exp = int32(new_exp);

src_1.ExposureTime = Exp;
src_2.ExposureTime = Exp;


if length(name1) > 1
    I1 = getsnapshot(vid_1);
    I1 = rotMvCorrect(I1,Ang,S_gamma,T);
    
    imshow(I1,[],'Parent',handles.axes_cam_1)
else
    I1 = [];
end

if length(name2) > 1
    I2 = getsnapshot(vid_2);
    I2 = rotMvCorrect(I2,Ang,S_gamma,T);
    
    imshow(I2,[],'Parent',handles.axes_cam_2)
else
    I2 = [];
end

answer = questdlg('Do you want to save the images from cam_1 & 2?', ...
	'Save Images 1&2 dialog', ...
	'Yes','No','Yes');

if strcmp(answer,'Yes')
    ss = ss+1;
    
    if ss==1
        selected_dir = uigetdir('F:\Image data');
    end
    
    if ss == 1 || ss == 3
        imwrite(I1,strcat(selected_dir,'\',num2str(ss*2),'.tif'));
        imwrite(I2,strcat(selected_dir,'\',num2str((ss*2) - 1),'.tif'));
    elseif ss == 2 || ss ==4
        imwrite(I1,strcat(selected_dir,'\',num2str((ss*2) - 1),'.tif'));
        imwrite(I2,strcat(selected_dir,'\',num2str(ss*2),'.tif'));
    end
    
    set(handles.nImage_12,'String',ss);
    
end
if ss == 4
    set(handles.nImage_12,'BackgroundColor','red')
    handlesArray = [handles.capture_12, handles.nImage_12];
    set(handlesArray, 'Enable', 'off');
end

Dim_12 = [2464/binn,2056/binn,3];
% Dim_3 = [4096/binn,2176/binn,3];

       if length(name1)>1
           hImage_1=image(zeros(Dim_12), 'Parent', handles.axes_cam_1);

%             setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_1,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_1, hImage_1);
       end
       
       if length(name2)>1
           hImage_2=image(zeros(Dim_12), 'Parent', handles.axes_cam_2);

%             setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCC);
            setappdata(hImage_2,'UpdatePreviewWindowFcn',@mypreview_fcn_rotCCBin);
            preview(vid_2, hImage_2);
       end


% --- Executes on selection change in binningPop.
function binningPop_Callback(hObject, eventdata, handles)
% hObject    handle to binningPop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global binn
contents = cellstr(get(hObject,'String'));
binn = str2num(contents{get(hObject,'Value')});
% Hints: contents = cellstr(get(hObject,'String')) returns binningPop contents as cell array
%        contents{get(hObject,'Value')} returns selected item from binningPop


% --- Executes during object creation, after setting all properties.
function binningPop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to binningPop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function revTime_Callback(hObject, eventdata, handles)
% hObject    handle to revTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global rev_time delay_time n_images3

rev_time = str2double(get(hObject,'String'));
% rev_time = round(rev_time);

%     rev_time = round((23959*motor_speed^(-1.022)),2); %T = 23959P^(-1.022) from stats
    delay_time = round((rev_time/n_images3),4);


% Hints: get(hObject,'String') returns contents of revTime as text
%        str2double(get(hObject,'String')) returns contents of revTime as a double


% --- Executes during object creation, after setting all properties.
function revTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to revTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Image_one.
function Image_one_Callback(hObject, eventdata, handles)
% hObject    handle to Image_one (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global count Images n_images3

n_1 = count;
n_2 = n_images3-n_1+1;

Im_ordered = cell(1,n_images3);

     if n_1 == n_images3
        Im_ordered{1} = Images{n_images3};
        Im_ordered(2:n_images3) = Images(1:(n_images3-1));
     elseif n_1 == 1
        Im_ordered = Images;
     else
        Im_ordered(1:n_2) = Images(n_1:n_images3);%images after selected #1
        Im_ordered((n_2+1):n_images3) = Images(1:(n_1-1));%images before selected #1
     end

Images = Im_ordered;

count = 1;
        imshow(Images{1},[],'Parent',handles.axes_cam_3)
        set(handles.imageNumber,'String',num2str(1));
drawnow


% --- Executes on button press in Cap_new.
function Cap_new_Callback(hObject, eventdata, handles)
% hObject    handle to Cap_new (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global count ss

ss = 0;
count = 1;

       
set(handles.nImage_12,'BackgroundColor','white')
handlesArray = [handles.capture_12, handles.nImage_12];
set(handlesArray, 'Enable', 'on');
set(handles.nImage_12,'String',ss);
