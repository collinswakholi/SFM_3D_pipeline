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

% Last Modified by GUIDE v2.5 08-Feb-2017 15:54:05

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
addpath('C:\Users\Kwon\Desktop\SFM_16a\New_gui_64');

CheckIcon = imread('Check.jpg');
StopIcon = imread('Stop.jpg');
PauseIcon = imread('Pause.jpg');


    set(handles.pushbutton12, 'Enable', 'off')
    set(handles.pushbutton7, 'Enable', 'off')
    set(handles.pushbutton8, 'Enable', 'off')
    set(handles.pushbutton9, 'Enable', 'off')
    set(handles.pushbutton19, 'Enable', 'off')
    set(handles.pushbutton18, 'Enable', 'off')
    set(handles.pushbutton20, 'Enable', 'off')
    set(handles.pushbutton21, 'Enable', 'off')
    set(handles.pushbutton13, 'Enable', 'off')
    set(handles.pushbutton14, 'Enable', 'off')
    set(handles.pushbutton3, 'Enable', 'off')
    set(handles.pushbutton4, 'Enable', 'off')
    set(handles.togglebutton1, 'Enable', 'off')
    set(handles.togglebutton2, 'Enable', 'off')
    
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
global CheckIcon


% load library
loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 

a=libfunctions('EziMOTIONPlusRx64');
% 
% if isempty(a)==0
%     msgbox('Load Completed','Success','custom',CheckIcon);
%     
    set(handles.pushbutton3, 'Enable', 'on')
    set(handles.pushbutton4, 'Enable', 'on')
%     
% else
%     msgbox('Cannot load Library','Error','Error')
% end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global CheckIcon

unloadlibrary('EziMOTIONPlusRx64');
 msgbox('Unload Completed','Success','custom',CheckIcon);


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


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global PortN Baudrate CheckIcon SlaveN

calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(PortN), int32(Baudrate));

% 
% % port open
% calllib('EziMOTIONPlusR', 'FAS_OpenPort',int32(PortN), int32(Baudrate));
% 
% % slave select
% calllib('EziMOTIONPlusR', 'FAS_AttachSlave',int32(PortN), int32(SlaveN));

% check slave
Check=calllib('EziMOTIONPlusRx64', 'FAS_IsSlaveExist',int32(PortN), int32(SlaveN));


% check connection

if Check==1
    
    msgbox('Connect Successfully!','Success','custom',CheckIcon);
    
    set(handles.togglebutton1, 'Enable', 'on')
    
    set(handles.togglebutton1, 'BackgroundColor', [0.75 0.39 0.39]);
    
else
    msgbox('Cannot Connect','Error','Error')
    
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global PortN

calllib('EziMOTIONPlusRx64', 'FAS_Close',int32(PortN));


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1
global ServoS PortN SlaveN

% set(handles.connection, 'Enable', 'off')
% set(handles.pushbutton2, 'Enable', 'on')
% set(handles.pushbutton3, 'Enable', 'on')

if get(hObject, 'Value')
    ServoS=1;
    
    set(handles.pushbutton12, 'BackgroundColor', [0 0.797 0.398]);
    set(handles.pushbutton7, 'BackgroundColor', [0 0.797 0.398]);
    set(handles.pushbutton8, 'BackgroundColor', [0 0.797 0.398]);
    set(handles.pushbutton9, 'BackgroundColor', [0.996 0.398 0.398]);
    set(handles.togglebutton2, 'BackgroundColor', [0.598 0.598 0.598]);
    set(handles.pushbutton13, 'BackgroundColor', [0.996 0.797 0.598]);
    set(handles.pushbutton14, 'BackgroundColor', [0.398 0.598 0.999]);
    set(handles.pushbutton12, 'Enable', 'on')
    set(handles.pushbutton14, 'Enable', 'on')
    set(handles.pushbutton7, 'Enable', 'on')
    set(handles.pushbutton8, 'Enable', 'on')
    set(handles.pushbutton9, 'Enable', 'on')
    set(handles.pushbutton13, 'Enable', 'on')
    set(handles.togglebutton2, 'Enable', 'on')
    
else
    ServoS=0;
    
    set(handles.pushbutton12, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton7, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton8, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton9, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.togglebutton2, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton13, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton14, 'BackgroundColor', [0.199 0.199 0.199]);
    set(handles.pushbutton14, 'Enable', 'off')
    set(handles.pushbutton12, 'Enable', 'off')
    set(handles.pushbutton7, 'Enable', 'off')
    set(handles.pushbutton8, 'Enable', 'off')
    set(handles.pushbutton9, 'Enable', 'off')
    set(handles.pushbutton13, 'Enable', 'off')
    set(handles.togglebutton2, 'Enable', 'off')
end
  

calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(PortN), int32(SlaveN), int32(ServoS));


% --- Executes during object creation, after setting all properties.
function togglebutton1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called






function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
global Position_ABS

Position_ABS=str2double(get(hObject, 'string'));
Position_ABS=1100000/360*Position_ABS;


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Position_ABS Velocity_ABS PortN SlaveN ABS_Ro

Position_ABS1=Position_ABS*ABS_Ro;

calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
    , int32(Position_ABS1), int32(Velocity_ABS));



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
global Position_INC

Position_INC=str2double(get(hObject, 'string'));
Position_INC=1100000/360*Position_INC;


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Position_INC Velocity_INC PortN SlaveN INC_Ro

ParamVal=lib.pointer('uint32Ptr',1);

Position_INC1=Position_INC*INC_Ro;

calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(Position_INC1), int32(Velocity_INC));



% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global PortN SlaveN StopIcon Auto_stop

calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(PortN), int32(SlaveN));

msgbox('Emergency Stop!','Stop','custom',StopIcon);

Auto_stop=1;





% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton2
global PortN SlaveN PauseIcon Auto_pause

if get(hObject, 'Value')
    
    Pause=1;
    calllib('EziMOTIONPlusRx64', 'FAS_MovePause', int32(PortN), int32(SlaveN), int32(Pause));
    msgbox('Emergency Pause!','Pause','custom',PauseIcon);
    Auto_pause=1;
    
else
    
    Pause=0;
    calllib('EziMOTIONPlusRx64', 'FAS_MovePause', int32(PortN), int32(SlaveN), int32(Pause));
    
end






% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3
global INC_Ro

switch get(hObject,'Value')

    case 1
        INC_Ro=1;
        
    case 2
        INC_Ro=-1;
end



% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4
global ABS_Ro

switch get(hObject,'Value')

    case 1
        ABS_Ro=1;
        
    case 2
        ABS_Ro=-1;
end


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Position_auto Velocity_auto PortN SlaveN Auto_stop StopIcon Auto_Ro

Auto_stop=0;
step=0;

Position_auto1=Position_auto*Auto_Ro;


Timer=str2double(get(handles.Timer,'string'));
Delay=str2double(get(handles.Delay,'string'));

cycle=0;

while cycle<1100000
    
calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(Position_auto1), int32(Velocity_auto));

step=step+1;
fprintf('%d Step\n',step);

% waiting time
pause(Delay) 

if Auto_stop==1
    
    calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(PortN), int32(SlaveN));

    break;
    
end


if abs(cycle) < 1100000
    
    pause(Timer)

    cycle=abs(cycle)+abs(Position_auto1);
        
end

end


calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
    , int32(0), int32(200000));



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double

global Position_auto

Position_auto=str2double(get(hObject, 'string'));
Position_auto=1100000/360*Position_auto;


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5
global Auto_Ro

switch get(hObject,'Value')

    case 1
        Auto_Ro=1;
        
    case 2
        Auto_Ro=-1;
end


% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Timer_Callback(hObject, eventdata, handles)
% hObject    handle to Timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Timer as text
%        str2double(get(hObject,'String')) returns contents of Timer as a double


% --- Executes during object creation, after setting all properties.
function Timer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global PortN SlaveN

calllib('EziMOTIONPlusRx64', 'FAS_ServoAlarmReset', int32(PortN), int32(SlaveN));


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6
global Velocity_INC

switch get(hObject,'Value')

    case 1
        Velocity_INC=50000;
        
    case 2
        Velocity_INC=100000;
        
    case 3
        Velocity_INC=150000;
        
    case 4
        Velocity_INC=200000;
end



% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7
global Velocity_ABS

switch get(hObject,'Value')

    case 1
        Velocity_ABS=50000;
        
    case 2
        Velocity_ABS=100000;
        
    case 3
        Velocity_ABS=150000;
        
    case 4
        Velocity_ABS=200000;
end


% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8
global Velocity_auto

switch get(hObject,'Value')

    case 1
        Velocity_auto=50000;
        
    case 2
        Velocity_auto=100000;
        
    case 3
        Velocity_auto=150000;
        
    case 4
        Velocity_auto=200000;
end


% --- Executes during object creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu9.
function popupmenu9_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu9 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu9

global PortN

switch get(hObject,'Value')

    case 1
        PortN=1;        
    case 2
        PortN=2;
    case 3
        PortN=3;
    case 4
        PortN=4;
    case 5
        PortN=5;
    case 6
        PortN=6;
    case 7
        PortN=7;
    case 8
        PortN=8;
end

% --- Executes during object creation, after setting all properties.
function popupmenu9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu10.
function popupmenu10_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu10 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu10

global SlaveN

switch get(hObject,'Value')

    case 1
        SlaveN=0;        
    case 2
        SlaveN=1;
    case 3
        SlaveN=2;
    case 4
        SlaveN=3;
    case 5
        SlaveN=4;
    case 6
        SlaveN=5;
    case 7
        SlaveN=6;
    case 8
        SlaveN=7;
    case 9
        SlaveN=8;        
    case 10
        SlaveN=9;
    case 11
        SlaveN=10;
    case 12
        SlaveN=11;
    case 13
        SlaveN=12;
    case 14
        SlaveN=13;
    case 15
        SlaveN=14;
    case 16
        SlaveN=15;
end


% --- Executes during object creation, after setting all properties.
function popupmenu10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu11.
function popupmenu11_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu11 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu11


global Baudrate

switch get(hObject,'Value')

    case 1
        Baudrate=9600;        
    case 2
        Baudrate=38400;
    case 3
        Baudrate=57600;
    case 4
        Baudrate=115200;
    case 5
        Baudrate=128000;
end



% --- Executes during object creation, after setting all properties.
function popupmenu11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Delay_Callback(hObject, eventdata, handles)
% hObject    handle to Delay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Delay as text
%        str2double(get(hObject,'String')) returns contents of Delay as a double


% --- Executes during object creation, after setting all properties.
function Delay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Delay (see GCBO)
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
global PortN SlaveN

calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
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


hImage=image(zeros(2712, 3384, 3), 'Parent', handles.axes2);

preview(vid, hImage);


    set(handles.pushbutton19, 'Enable', 'on')
    set(handles.pushbutton18, 'Enable', 'on')
    set(handles.pushbutton20, 'Enable', 'on')
    set(handles.pushbutton21, 'Enable', 'on')
    
    set(handles.pushbutton20, 'BackgroundColor', [0.598 0.797 0.996]);
    set(handles.pushbutton21, 'BackgroundColor', [0.996 0.598 0.398]);



% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid
global name
% global hImage;

start(vid);
% stoppreview(vid);

filename=strcat('IM',name,'.jpg');

capture1=getsnapshot(vid);

imwrite(capture1, char(filename));

pause(2)

imshow(capture1);




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


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Position_auto Velocity_auto PortN SlaveN Acqui_stop StopIcon Auto_Ro ...
    vid name

Acqui_stop=0;
step=0;
start(vid);
i=0;

Position_auto1=Position_auto*Auto_Ro;


Timer=str2double(get(handles.Timer,'string'));
Delay=str2double(get(handles.Delay,'string'));

cycle=0;


while cycle<1100000
    
calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(Position_auto1), int32(Velocity_auto));

step=step+1;
fprintf('%d Step\n',step);

% waiting time
pause(Delay) 

if Acqui_stop==1
    
    calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(PortN), int32(SlaveN));

    break;
    
end



if abs(cycle) < 1100000
    
    pause(Timer)

    cycle=abs(cycle)+abs(Position_auto1);
    
    
    capture1=getsnapshot(vid);

    filename=strcat('IM',name,'_', num2str(i) ,'.jpg');
    imwrite(capture1, char(filename));
    pause(1)

    imshow(capture1);
    pause(1)
    
    i=i+1;


end

end


calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
    , int32(0), int32(100000));


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Acqui_stop

Acqui_stop=1;
