%% Train Stop Sign Detector
% Load the positive samples data from a MAT file. The file contains
% a table specifying bounding boxes for several object categories.
% The table was exported from the Training Image Labeler app.
%%
% Load positive samples.
%load('stopSignsAndCars.mat');
%%
% Select the bounding boxes for stop signs from the table.
positiveInstances = Cucumbers(:,1:2);
%%
% Add the image directory to the MATLAB path.
imDir = fullfile('/home','chiel', 'Bureaublad', 'tmp', 'good');
addpath(imDir);
%%
% Specify the foler for negative images.
negativeFolder = fullfile('/home','chiel', 'Bureaublad', 'tmp', 'bad');
%%
% Create an |imageDatastore| object containing negative images.
negativeImages = imageDatastore(negativeFolder);
%%
% Train a cascade object detector called 'stopSignDetector.xml'
% using HOG features.
% NOTE: The command can take several minutes to run.
trainCascadeObjectDetector('test.xml',positiveInstances, ...
    negativeFolder,'FalseAlarmRate',0.01,'NumCascadeStages', 20);
%%
% Use the newly trained classifier to detect a stop sign in an image.
detector = vision.CascadeObjectDetector('test.xml');
%%
% Read the test image.
img = imread('/home/chiel/Bureaublad/tmp/good/08-07-39-133187.jpg');
tic;
%%
% Detect a stop sign.
bbox = step(detector,img); 
toc
%%
% Insert bounding box rectangles and return the marked image.
 detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'cucumber');
%%
% Display the detected stop sign.
figure; imshow(detectedImg);
%%
% Remove the image directory from the path.
rmpath(imDir); 