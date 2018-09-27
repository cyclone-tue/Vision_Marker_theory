clear all; close all; clc;
tic

img = imread('1.jpg');
mark = imread('marker.png');

img = rgb2hsv(img);
img = img(:,:,3);
img = edge(img, 'Canny');

mark = rgb2hsv(mark);
mark = mark(:,:,3);

mark = imresize(mark,[size(img,1),size(img,2)]);
% mark = edge(mark, 'Canny');

marker = cell(1,10);
n_sizes = 4;
for i=1:n_sizes
    x = round(size(img,1)/i);
    y = round(size(img,2)/i);
    marker{i} = imresize(mark,[x,y]);
end
n_angles = 16;
for i=1:n_sizes
    for j=1:n_angles
        marker2{i,j} = imrotate(marker{i},(360/n_angles)*j,'crop');
        marker2{i,j} = edge(marker2{i,j},'Canny');
        figure;
        imshow(marker2{i,j});
    end
end
for i=1:n_sizes
    for j=1:n_angles
        c{i,j} = normxcorr2(marker2{i,j},img);
    end
    max_val{i} = max(max(c{i,j}));
%     peaks{i} = [find(c{i,:}==max(max(c{i,j})),'first') find(c{:,j}==max(max(c{i,j})))];
end

[ypeak, xpeak] = find(c{i,j}==max(c{i,j}));
yoffSet = ypeak-size(img,1);
xoffSet = xpeak-size(img,2);
figure;
imshow(img);
hold on;
% plot(xpeak, ypeak, 'ro', 'linewidth', 3);
% imrect(gca, [xoffSet+1, yoffSet+1, size(marker,2), size(marker,1)]);

toc