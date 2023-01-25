% Luke Davidson
% EECE 5554
% Lab 5 - Feature Detection and Image Layering
close all

%% Part 1 - Load Images
% Load images.
% buildingDir = fullfile(toolboxdir('vision'),'visiondata','building');

% buildingDir = fullfile('/Users','lukedavidson','Downloads','northeastern','eece5554_rsn','Lab5_pics',{'lab5_im1.jpg';'lab5_im2.jpg';'lab5_im3.jpg';'lab5_im4.jpg';'lab5_im5.jpg'});
% buildingDir = fullfile('/Users','lukedavidson','Downloads','northeastern','eece5554_rsn','Lab5_pics',{'lab5_im6.jpg';'lab5_im7.jpg';'lab5_im8.jpg';'lab5_im9.jpg';'lab5_im10.jpg'});
buildingDir = fullfile('/Users','lukedavidson','Downloads','northeastern','eece5554_rsn','Lab5_pics','fifteen_overlap',{'IMG_4075.jpg';'IMG_4076.jpg';'IMG_4077.jpg';'IMG_4078.jpg';'IMG_4079.jpg'});
buildingScene = imageDatastore(buildingDir);

% Display images to be stitched.
montage(buildingScene.Files)



%% Part 2a - Image Pairs (Harris)

% Read the first image from the image set.
I = readimage(buildingScene,1);


% Initialize features for I(1)
grayImage = im2gray(I);
N = 10000;

% harris(grayImage)
[points_y,points_x,m] = harris(grayImage,N);
locations = [points_x, points_y];
points = cornerPoints(locations,'Metric',m);
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
numImages = numel(buildingScene.Files);
tforms(numImages) = projective2d(eye(3));

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
%     window_bounds_prev = window_bounds
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = readimage(buildingScene, n);
    
    % Convert image to grayscale.
    grayImage = im2gray(I);    
    
    % Save image size.
    imageSize(n,:) = size(grayImage);
    
%     harris(grayImage)
    [points_y,points_x,m] = harris(grayImage,N);
    locations = [points_x,points_y];
    points = cornerPoints(locations,'Metric',m);
    [features, points] = extractFeatures(grayImage, points);
    
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
    
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform2D(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 4000);
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

% Compute the output limits for each transform.
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end

avgXLim = mean(xlim, 2);
[~,idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));
for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end



%% Part 3 - Initialize Panorama

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits. 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I);



%% Part 4 - Create Panorama

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    I = readimage(buildingScene, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
panorama = imrotate(panorama,-90);
imshow(panorama)