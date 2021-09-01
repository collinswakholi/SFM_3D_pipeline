function varargout = motion_control(varargin)
% MOTION_CONTROL MATLAB code for motion_control.fig
%      MOTION_CONTROL, by itself, creates a new MOTION_CONTROL or raises the existing
%      singleton*.
%
%      H = MOTION_CONTROL returns the handle to a new MOTION_CONTROL or the handle to
%      the existing singleton*.
%
%      MOTION_CONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOTION_CONTROL.M with the given input arguments.
%
%      MOTION_CONTROL('Property','Value',...) creates a new MOTION_CONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before motion_control_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to motion_control_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help motion_control

% Last Modified by GUIDE v2.5 10-Feb-2017 14:37:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @motion_control_OpeningFcn, ...
                   'gui_OutputFcn',  @motion_control_OutputFcn, ...
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


% --- Executes just before motion_control is made visible.
function motion_control_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to motion_control (see VARARGIN)

% Choose default command line output for motion_control
handles.output = hObject;
global CheckIcon StopIcon PauseIcon

% addpath
addpath('C:\Users\Owner\Desktop\New_gui_64');

CheckIcon = imread('Check.jpg');
StopIcon = imread('Stop.jpg');
PauseIcon = imread('Pause.jpg');

    set(handles.pushbutton22, 'Enable', 'off')
    set(handles.pushbutton21, 'Enable', 'off')
    set(handles.pushbutton19, 'Enable', 'off')
    set(handles.pushbutton14, 'Enable', 'off')
        set(handles.edit13, 'Enable', 'off')
    
axes(handles.axes2);
axes(handles.axes3);
    
% axes(handles.axes1)
% imshow(imread('logo.png'));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes motion_control wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = motion_control_OutputFcn(hObject, eventdata, handles) 
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

PortN=3;
Baudrate=115200;
SlaveN=1;
ServoS=1;

% load library
a=libfunctions('EziMOTIONPlusRx64');

if isempty(a)==0;
    fprintf('Exist Library \n');
else
    loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 
    fprintf('Load Complete!\n');
end

pause(1)
calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(PortN), int32(Baudrate));
fprintf('Connect Motor! \n');

pause(1)
calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(PortN), int32(SlaveN), int32(ServoS));
fprintf('Servo On! \n');
    set(handles.pushbutton14, 'Enable', 'on')






% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(3), int32(1)...
    , int32(0), int32(100000));


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global vid;
global hImage;

% 
% % vid = videoinput('gige', 1);
vid = videoinput('gige', 1, 'BayerRG8');

% vid.ROIPosition = [0 0 1700 1712];

hImage=image(zeros(2712, 3384, 3), 'Parent', handles.axes2);

preview(vid, hImage);


    set(handles.pushbutton17, 'Enable', 'off')
    set(handles.pushbutton19, 'Enable', 'on')
    set(handles.edit13, 'Enable', 'on')
    set(handles.pushbutton22, 'Enable', 'on')
    set(handles.pushbutton21, 'Enable', 'on')
    
    set(handles.pushbutton22, 'BackgroundColor', [0.598 0.797 0.996]);
    set(handles.pushbutton21, 'BackgroundColor', [0.996 0.598 0.398]);






function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double

global name
name=get(hObject, 'String');

% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vid
stoppreview(vid);
delete(vid);

    set(handles.pushbutton17, 'Enable', 'on')
    set(handles.pushbutton19, 'Enable', 'off')
    set(handles.edit13, 'Enable', 'off')
    set(handles.pushbutton22, 'Enable', 'off')
    set(handles.pushbutton21, 'Enable', 'off')
    
    

% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Acqui_stop PortN SlaveN

calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(PortN), int32(SlaveN));

msgbox('Emergency Stop!','Stop');

Acqui_stop=1;


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Acqui_stop vid name

Velocity_auto=12220;
PortN=3;
SlaveN=1;

Acqui_stop=0;
start(vid);
capture1=getsnapshot(vid);

filename=strcat('IM',name,'_0.jpg');
imwrite(capture1, char(filename));
pause(1) 


axes(handles.axes3);
imshow(capture1);
    

h = waitbar(0,'Please wait...');

for i=1:90
        
        if Acqui_stop==1;
            break
        end
    
    calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(12222), int32(Velocity_auto));

    
    pause(1.5)
    
    
    capture1=getsnapshot(vid);
    filename=strcat('IM',name,'_', num2str(i) ,'.jpg');
    imwrite(capture1, char(filename));

    imshow(capture1);
    
    waitbar(i/90,h)
        
    if Acqui_stop==1;
        break
    end
    
    
end
close(h);

if Acqui_stop==0

    calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
    , int32(0), int32(100000));

end


% --- Executes during object creation, after setting all properties.
function pushbutton17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
