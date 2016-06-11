%%%
% PID Hand Tuner
% By Thanasis Georgiou <contact@thgeorgiou.com>
% (https://github.com/sakisds/mat-PID-hand-tuner)
% Licenced under the MIT license.
%%%

function varargout = pid_hand_tuner(varargin)
% PID_HAND_TUNER MATLAB code for pid_hand_tuner.fig
%      PID_HAND_TUNER, by itself, creates a new PID_HAND_TUNER or raises the existing
%      singleton*.
%
%      H = PID_HAND_TUNER returns the handle to a new PID_HAND_TUNER or the handle to
%      the existing singleton*.
%
%      PID_HAND_TUNER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PID_HAND_TUNER.M with the given input arguments.
%
%      PID_HAND_TUNER('Property','Value',...) creates a new PID_HAND_TUNER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pid_hand_tuner_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pid_hand_tuner_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pid_hand_tuner

% Last Modified by GUIDE v2.5 11-Jun-2016 14:07:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pid_hand_tuner_OpeningFcn, ...
                   'gui_OutputFcn',  @pid_hand_tuner_OutputFcn, ...
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


% --- Executes just before pid_hand_tuner is made visible.
function pid_hand_tuner_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pid_hand_tuner (see VARARGIN)

% Complain if parameters are not transfer functions
if ~isa(varargin{1}, 'tf') || ~isa(varargin{2}, 'tf')
    errordlg(...
        'Parameters are not transfer functions!', ...
        'Invalid Input' ...
        );
    close();
end

% Store TFs
handles.system = varargin{1};
handles.feedback = varargin{2};

% Choose default command line output for pid_hand_tuner
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes pid_hand_tuner wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pid_hand_tuner_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbuttonRefresh.
function pushbuttonRefresh_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonRefresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Create PID
tfPID = pid( ...
    str2double(handles.editProportional.String), ...
    str2double(handles.editIntegral.String), ...
    str2double(handles.editDerivative.String) ...
);

% Create closed and open loop
tfCL = feedback(tfPID * handles.system, handles.feedback);
tfOL = tfPID * handles.system * handles.feedback;

% Get t_final and y_target
tFinal = str2num(handles.editTime.String);
yTarget = str2num(handles.editTarget.String);

% Calculate step response
stepOpts = stepDataOptions('StepAmplitude', yTarget);
[stepY, stepT] = step(tfCL, tFinal, stepOpts);

% Calculate final value
[num, den] = tfdata(tfCL);
s = sym('s');
T = poly2sym(cell2mat(num), s) / ...
    poly2sym(cell2mat(den), s);
finalValue = limit(T, s, 0);
finalValueError = (abs(finalValue - yTarget)/yTarget) * 100;

% Calculate step characteristics & update UI
stepInfo = stepinfo(tfCL);
[Gm, Pm, Wgm, Wpm] = margin(tfOL);
rows = [ ...
    {'Rise time', stepInfo.RiseTime, 's'}; ...
    {'Settling time', stepInfo.SettlingTime, 's'}; ...
    {'Overshoot', stepInfo.Overshoot, '%'}; ...
    {'Peak', stepInfo.Peak, 'abs'}; ...
    {'Min Y', stepInfo.SettlingMin, 'abs'}; ...
    {'Max Y', stepInfo.SettlingMax, 'abs'}; ...
    {'Final value', double(finalValue), 'abs'}; ...
    {'Final value error', double(finalValueError), '%'}; ...
    {'Gain margin', 20*log10(Gm), 'dB'}; ...
    {'Phase margin', Pm, 'deg'}; ...
    {'Gain margin freq.', Wgm, 'rad/s'}; ...
    {'Phase margin freq.', Wpm, 'rad/s'} ...
    ];
handles.tableCharacteristics.Data = rows;

% Calculate bode data
[mag, phase, freq] = bode(tfOL);
mag = 20*log10(mag(:));
phase = phase(:);

% Plot step
axes(handles.axesStep);
plot( ...
    stepT, stepY, ...
    [0 tFinal], [yTarget yTarget] ...
);

xlabel('Time');
ylabel('Output');
handles.axesStep.XLim = [0 tFinal];
handles.axesStep.YGrid = 'on';
handles.axesStep.XGrid = 'on';

% Plot bode
axes(handles.axesBodeMag);
semilogx(freq, mag);

xlabel('Angular Frequency (rad/s)');
ylabel('Magnitude (dB)');
grid on;
grid minor;

axes(handles.axesBodePhase);
semilogx(freq, phase);

xlabel('Angular Frequency (rad/s)');
ylabel('Phase shift (deg)');
grid on;
grid minor;

function editTime_Callback(hObject, eventdata, handles)
% hObject    handle to editTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

str = get(hObject, 'String');
if isempty(str2num(str))
    set(hObject, 'string', '0');
    warndlg('Input must be numerical');
else
    pushbuttonRefresh_Callback(hObject, eventdata, handles);
end

% --- Executes during object creation, after setting all properties.
function editTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editTarget_Callback(hObject, eventdata, handles)
% hObject    handle to editTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

str = get(hObject, 'String');
if isempty(str2num(str))
    set(hObject, 'string', '0');
    warndlg('Input must be numerical');
else
    pushbuttonRefresh_Callback(hObject, eventdata, handles);
end


% --- Executes during object creation, after setting all properties.
function editTarget_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function editProportional_Callback(hObject, eventdata, handles)
% hObject    handle to editProportional (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read the number and refresh the graphs
str = get(hObject, 'String');
num = str2num(str);
if isempty(num)
    set(hObject, 'string', '0');
    warndlg('Input must be numerical');
else
    % This is probably bad practice
    pushbuttonRefresh_Callback(hObject, eventdata, handles);
end;

% --- Executes during object creation, after setting all properties.
function editProportional_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editProportional (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonPA01.
function pushbuttonPA01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPA01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 0.1 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num + 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonAS01.
function pushbuttonAS01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonAS01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtract 0.1 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num - 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonPA1.
function pushbuttonPA1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 1 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num + 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonPS1.
function pushbuttonPS1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtract 1 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num - 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonPA10.
function pushbuttonPA10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPA10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 10 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num + 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonPS10.
function pushbuttonPS10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPS10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtract 10 to proportional and refresh graphs
str = get(handles.editProportional, 'String');
num = str2double(str);
set(handles.editProportional, 'String', num - 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


function editIntegral_Callback(hObject, eventdata, handles)
% hObject    handle to editIntegral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read the number and refresh the graphs
str = get(hObject, 'String');
num = str2num(str);
if isempty(num)
    set(hObject, 'string', '0');
    warndlg('Input must be numerical');
else
    % This is probably bad practice
    pushbuttonRefresh_Callback(hObject, eventdata, handles);
end;


% --- Executes during object creation, after setting all properties.
function editIntegral_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editIntegral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonIA01.
function pushbuttonIA01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIA01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 0.1 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num + 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonIS01.
function pushbuttonIS01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIS01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 0.1 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num - 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonIA1.
function pushbuttonIA1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 1 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num + 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);

% --- Executes on button press in pushbuttonIS1.
function pushbuttonIS1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 1 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num - 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonIA10.
function pushbuttonIA10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIA10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 10 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num + 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonIS10.
function pushbuttonIS10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonIS10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 10 to integral and refresh graphs
str = get(handles.editIntegral, 'String');
num = str2double(str);
set(handles.editIntegral, 'String', num - 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);



function editDerivative_Callback(hObject, eventdata, handles)
% hObject    handle to editDerivative (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read the number and refresh the graphs
str = get(hObject, 'String');
num = str2num(str);
if isempty(num)
    set(hObject, 'string', '0');
    warndlg('Input must be numerical');
else
    % This is probably bad practice
    pushbuttonRefresh_Callback(hObject, eventdata, handles);
end;

% --- Executes during object creation, after setting all properties.
function editDerivative_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDerivative (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonDA01.
function pushbuttonDA01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDA01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 0.1 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num + 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonDS01.
function pushbuttonDS01_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDS01 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 0.1 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num - 0.1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonDA1.
function pushbuttonDA1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 0.1 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num + 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonDS1.
function pushbuttonDS1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDS1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 1 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num - 1);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonDA10.
function pushbuttonDA10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDA10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Add 0.1 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num + 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);


% --- Executes on button press in pushbuttonDS10.
function pushbuttonDS10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonDS10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Subtracts 10 to derivative and refresh graphs
str = get(handles.editDerivative, 'String');
num = str2double(str);
set(handles.editDerivative, 'String', num - 10);
% This is probably bad practice
pushbuttonRefresh_Callback(hObject, eventdata, handles);
