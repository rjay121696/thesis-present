function varargout = main(varargin)
% MAIN MATLAB code for main.fig
%      MAIN, by itself, creates a new MAIN or raises the existing
%      singleton*.
%
%      H = MAIN returns the handle to a new MAIN or the handle to
%      the existing singleton*.
%
%      MAIN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN.M with the given input arguments.
%
%      MAIN('Property','Value',...) creates a new MAIN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before main_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to main_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help main

% Last Modified by GUIDE v2.5 28-Oct-2019 16:40:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @main_OpeningFcn, ...
                   'gui_OutputFcn',  @main_OutputFcn, ...
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


% --- Executes just before main is made visible.
function main_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to main (see VARARGIN)

% Choose default command line output for main
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes main wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = main_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(~, ~, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc

camera = webcam();
nnet = alexnet;
%%


while true
    picture = camera.snapshot;
    picture = imresize(picture, [227,227]);
    
   
    label = classify(nnet, picture);
    
    image(picture);
    
   
     if label == 'banana'
         set(handles.edit1, 'ForegroundColor', 'green', 'string', 'YES');
     set(handles.uipanel1, 'highlightcolor', 'g')
     % For Removing Background
            % 1
        Agray = colouredToGray(picture);
%       Agray = rgb2gray(picture);
     imageSize = size(picture);
     numRows = imageSize(1);
    numCols = imageSize(2);


       wavelengthMin = 4/sqrt(2);
    wavelengthMax = hypot(numRows,numCols);
    n = floor(log2(wavelengthMax/wavelengthMin));
    wavelength = 2.^(0:(n-2)) * wavelengthMin;

    deltaTheta = 45;
    orientation = 0:deltaTheta:(180-deltaTheta);

    g = gabor(wavelength,orientation);
    % 1
    gabormag = imgaborfilt(Agray,g);
    %2
    parfor i = 1:length(g)
    sigma = 0.5*g(i).Wavelength;
    K = 3;
    gabormag(:,:,i) = imgaussfilt(gabormag(:,:,i),K*sigma); 
    end
    %2
    %*
    X = 1:numCols;
    Y = 1:numRows;
    [X,Y] = meshgrid(X,Y);
    featureSet = cat(3,gabormag,X);
    featureSet = cat(3,featureSet,Y);
    %*
    %*1
    numPoints = numRows*numCols;
    X = reshape(featureSet,numRows*numCols,[]);
    %*1
    %*2
    X = bsxfun(@minus, X, mean(X));
    X = bsxfun(@rdivide,X,std(X));
    %*2
    %*3
    coeff = pca(X);
    feature2DImage = reshape(X*coeff(:,1),numRows,numCols);
    %*3
    %*3-1
    L = kmeans(X,2,'Replicates',5);
    %*3-1
    %3-2
	L = reshape(L,[numRows numCols]);
    %3-2
    
    %*3-3
    
    Aseg1 = zeros(size(picture),'like',picture);
%    Aseg2 = zeros(size(picture),'like',picture);
    BW = L == 2;
    BW = repmat(BW,[1 1 3]);
    Aseg1(BW) = picture(BW);
%      Aseg2(~BW) = picture(~BW);

%     foreground = bwareaopen(Aseg1(BW), 8);
    labelimage = bwlabel(Aseg1(BW), 8);
    measurements = regionprops(labelimage, 'BoundingBox');
   
    
    imshow(Aseg1);
    drawnow;
    
    %Detect Color Grren
   diff_im = imsubtract(picture(:,:,2), rgb2gray(picture)); 
      diff_im = medfilt2(diff_im, [3 3]);
      diff_im = imbinarize(diff_im,0.18);
      diff_im = bwareaopen(diff_im,300);
      bw = bwlabel(diff_im, 8);
      stats = regionprops(bw, 'BoundingBox', 'Centroid');
      
        imshow(picture)
      hold on
      for object = 1:length(stats)
          bb = stats(object).BoundingBox;
          bc = stats(object).Centroid;
          rectangle('Position',bb,'EdgeColor','g','LineWidth',2)
          plot(bc(1),bc(2), '-m+')
          a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
          set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
      end
      hold off
       drawnow;
         im_gray = rgb2gray(picture);
         holes = im_gray < 5;
        [~,count] = bwlabel(holes);
       
     if  ~isempty(stats)
          set(handles.edit2, 'ForegroundColor', 'g', 'string', 'YES');
          set(handles.edit3, 'ForegroundColor', 'g', 'string', 'YES');
      else isempty(stats)
          set(handles.edit2, 'ForegroundColor', 'red', 'string', 'NO');
          set(handles.edit3, 'ForegroundColor', 'red', 'string', 'YES this is an banana');
      end
   if  count > 0
          set(handles.edit4, 'ForegroundColor', 'g', 'string', count);
          set(handles.edit5, 'ForegroundColor', 'r', 'string', 'But this is not a Banana');
      else 
          set(handles.edit4, 'ForegroundColor', 'red', 'string', 'NO');
          set(handles.edit5, 'ForegroundColor', 'r', 'string', 'No this is not a banana');
     end
      
     else
        

      set(handles.edit1, 'ForegroundColor', 'red', 'string', 'NO');
     set(handles.uipanel1, 'highlightcolor', 'r');
     %Detect Color Grren
    diff_im = imsubtract(picture(:,:,2), rgb2gray(picture)); 
      diff_im = medfilt2(diff_im, [3 3]);
      diff_im = imbinarize(diff_im,0.18);
      diff_im = bwareaopen(diff_im,300);
      bw = bwlabel(diff_im, 8);
      stats = regionprops(bw, 'BoundingBox', 'Centroid');
      
        imshow(picture)
      hold on
      for object = 1:length(stats)
          bb = stats(object).BoundingBox;
          bc = stats(object).Centroid;
          rectangle('Position',bb,'EdgeColor','g','LineWidth',2)
          plot(bc(1),bc(2), '-m+')
          a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
          set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
      end
      hold off
       drawnow;
      if  ~isempty(stats)
          set(handles.edit2, 'ForegroundColor', 'g', 'string', 'YES');
          set(handles.edit3, 'ForegroundColor', 'red', 'string', 'But this is not a Banana');
      elseif isempty(stats)
          set(handles.edit2, 'ForegroundColor', 'red', 'string', 'NO');
          set(handles.edit3, 'ForegroundColor', 'red', 'string', 'this is not a banana');
      end
        im_gray = rgb2gray(picture);
         holes = im_gray < 5;
        [~,count] = bwlabel(holes);
     if  count > 0
          set(handles.edit4, 'ForegroundColor', 'g', 'string', count);
          set(handles.edit5, 'ForegroundColor', 'r', 'string', 'But this is not a Banana');
      else 
          set(handles.edit4, 'ForegroundColor', 'red', 'string', 'NO');
          set(handles.edit5, 'ForegroundColor', 'r', 'string', 'No this is not a banana');
     end
        
        
    %3-3
%     hold on 
%     parfor object = 1:length(measurements)
%         cc = measurements(object).BoundingBox;
%      
%     end
%     hold on
 
     
    
     %Detect Spots
%         set(handles.edit1, 'string', 'NO');
%       
%       [rows, columns, numberOfColorBands] = size(picture);
%       redChannel = picture(:, :, 1);
%     greenChannel = picture(:, :, 2);
%     blueChannel = picture(:, :, 3);
%     binaryImage = redChannel < 172;
%     binaryImage = imclearborder(binaryImage);
%     [labeledImage, numberOfSpots] = bwlabel(binaryImage);
% %         labelimage = bwlabel(binaryImage, 8);
%         measurements = regionprops(labeledImage, 'BoundingBox', 'Centroid');
%         
%         imshow(picture);
%       parfor object = 1:length(measurements)
%         cc = measurements(object).BoundingBox;
%         dd = measurements(object).Centroid;
%          circles('Position',cc,'EdgeColor','g','LineWidth',2)
%          plot(dd(1),dd(2), '-m+')
%          a = 
%          set(handles.edit2, 'string', numberOfSpots);
%         
%     end
%     hold on
%     drawnow;
     end
end
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(~, ~, ~)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all; close all; clc;



function edit1_Callback(~, ~, ~)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, ~, ~)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(~, ~, ~)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, ~, ~)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


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



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


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



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
