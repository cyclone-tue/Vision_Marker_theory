close all
clear all

vid = videoinput('winvideo', 3, 'I420_640x480');
src = getselectedsource(vid);
vid.ReturnedColorspace = 'rgb';
vid.FramesPerTrigger = 1;

figure;
while 1
    start(vid)
    camdata = getdata(vid);
    img = rgb2hsv(camdata);
    img = img(:,:,3);
    Harris = detectHarrisFeatures(img);
    k=7;
    strongestHarris = Harris.selectStrongest(k);
    centreharris = [sum(strongestHarris.Location(:,1))/k sum(strongestHarris.Location(:,2))/k];
    imshow(img); 
    hold on; 
    plot(Harris.Location(:,1),Harris.Location(:,2) ,'bo','LineWidth',2); 
    plot(centreharris(1), centreharris(2),'r+','LineWidth',10);
    hold off;
    
end