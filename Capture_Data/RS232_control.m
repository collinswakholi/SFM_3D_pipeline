function varargout = RS232_control(varargin)
% RS232_CONTROL MATLAB code for RS232_control.fig
%      RS232_CONTROL, by itself, creates a new RS232_CONTROL or raises the existing
%      singleton*.
%
%      H = RS232_CONTROL returns the handle to a new RS232_CONTROL or the handle to
%      the existing singleton*.
%
%      RS232_CONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RS232_CONTROL.M with the given input arguments.
%
%      RS232_CONTROL('Property','Value',...) creates a new RS232_CONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RS232_control_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RS232_control_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RS232_control

% Last Modified by GUIDE v2.5 09-Feb-2017 19:25:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RS232_control_OpeningFcn, ...
                   'gui_OutputFcn',  @RS232_control_OutputFcn, ...
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


% --- Executes just before RS232_control is made visible.
function RS232_control_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RS232_control (see VARARGIN)

% Choose default command line output for RS232_control
handles.output = hObject;


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RS232_control wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RS232_control_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
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
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global obj1

obj1 = instrfind('Type', 'serial', 'Port', 'COM1', 'Tag', '');

% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = serial('COM1');
else
    fclose(obj1);
    obj1 = obj1(1)
end

% Connect to instrument object, obj1.
fopen(obj1);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global obj1

fclose(obj1);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a obj1

fprintf(obj1, a);
