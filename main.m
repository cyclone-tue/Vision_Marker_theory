clear all; close all; clc;
tic

img = imread('1.jpg');
% img2 = imread('2.jpg');
marker = imread('marker.png');

img = rgb2hsv(img);
img = img(:,:,3);
marker = rgb2hsv(marker);
marker = marker(:,:,3);
x = round(size(img,1)/3);
y = round(size(img,2)/3);
marker = imresize(marker,[x,y]);
Harris = detectHarrisFeatures(img);
Canny = edge(img, 'Canny');

SURF_regions = detectSURFFeatures(img);
% Feature_regions = detectfeatures(img);

[SURF_features, SURF_ipoints] = extractFeatures(img, SURF_regions);

% [Features_features, Feature_ipoints] = extractFeatures(img, Feature_regions);
%  
% matched_features = matchFeatures(SURF_features,features{2},'MatchThreshold', 80,'MaxRatio', 0.4,'Metric', 'SSD', 'Method','Exhaustive', 'Unique', true);
%     
% matchedPoints{1} = ipoints{1}(matched_features(:,1));
% matchedPoints{2} = ipoints{2}(matched_features(:,2));
% 
% figure;
% showMatchedFeatures(imgCell{k},imgCell{k+1},matchedPoints{1},matchedPoints{2});
k=7;
strongestHarris = Harris.selectStrongest(k);
strongestSURF = SURF_ipoints.selectStrongest(k);

centreharris = [sum(strongestHarris.Location(:,1))/k sum(strongestHarris.Location(:,2))/k];
centreSURF = [sum(strongestSURF.Location(:,1))/k sum(strongestSURF.Location(:,2))/k];

figure;
imshow(img); hold on; plot(Harris.Location(:,1),Harris.Location(:,2) ,'bo','LineWidth',2); 
plot(centreharris(1), centreharris(2),'r+','LineWidth',10);
hold off;
title('Harris');

figure;
imshow(img); hold on; plot(Harris.selectStrongest(k)); 
plot(centreharris(1), centreharris(2),'r+','LineWidth',10);
hold off;
title('Harris');

figure; 
imshow(Canny);
title('Canny');

figure;
imshow(img); hold on; plot(selectStrongest(SURF_ipoints, k));
plot(centreSURF(1), centreSURF(2),'r+','LineWidth',10);hold off;
title('SURF_IP');

c = normxcorr2(marker,img);
% figure;
% surf(c), shading flat

% [ypeak, xpeak] = find(c==max(c(:)));
% yoffSet = ypeak-size(marker,1);
% xoffSet = xpeak-size(marker,2);
% figure;
% imshow(img);
% imrect(gca, [xoffSet+1, yoffSet+1, size(marker,2), size(marker,1)]);

toc