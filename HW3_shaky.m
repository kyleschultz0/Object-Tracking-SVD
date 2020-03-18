clc; clear all; close all

%% shaky case
%% Create .avi from given .mat file to use with CV toolbox

load("cam1_2.mat");
videoFWriter = vision.VideoFileWriter('cam1_2.avi');
size1_1 = size(vidFrames1_2);

for i=1:size1_1(4)
  videoFrame = vidFrames1_2(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam2_2.mat");
videoFWriter = vision.VideoFileWriter('cam2_2.avi');
size2_1 = size(vidFrames2_2);

for i=1:size2_1(4)
  videoFrame = vidFrames2_2(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam3_2.mat");
videoFWriter = vision.VideoFileWriter('cam3_2.avi');
size3_1 = size(vidFrames3_2);

for i=1:size3_1(4)
  videoFrame = vidFrames3_2(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

%%  Create object region

videoFileReader1_1 = vision.VideoFileReader('cam1_2.avi');
videoPlayer1_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame1_1 = videoFileReader1_1();
figure; imshow(objectFrame1_1);
objectRegion1_1=round(getPosition(imrect));

videoFileReader2_1 = vision.VideoFileReader('cam2_2.avi');
videoPlayer2_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame2_1 = videoFileReader2_1();
figure; imshow(objectFrame2_1);
objectRegion2_1=round(getPosition(imrect));

videoFileReader3_1 = vision.VideoFileReader('cam3_2.avi');
videoPlayer3_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame3_1 = videoFileReader3_1();
figure; imshow(objectFrame3_1);
objectRegion3_1=round(getPosition(imrect));




%% Track Points

% Camera 1 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame1_1),'ROI',objectRegion1_1, 'MinQuality', 0.001);
tracker = vision.PointTracker('MaxBidirectionalError',3);
initialize(tracker,points.Location,objectFrame1_1);

i = 0;
while ~isDone(videoFileReader1_1)
    i = i+1;
    frame = videoFileReader1_1();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x1_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y1_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer1_1(out);
    pause(0.01)
end

for i = 1:size1_1(4)-1
    imshow(vidFrames1_2(:, :, :, i))
    hold on
    plot(x1_1(i), y1_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 2 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame2_1),'ROI',objectRegion2_1, 'MinQuality', 0.001);
tracker = vision.PointTracker('MaxBidirectionalError',80);
initialize(tracker,points.Location,objectFrame2_1);

i = 0;
while ~isDone(videoFileReader2_1)
    i = i+1;
    frame = videoFileReader2_1();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x2_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y2_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer2_1(out);
    pause(0.01)
end

for i = 1:size2_1(4)-1
    imshow(vidFrames2_2(:, :, :, i))
    hold on
    plot(x2_1(i), y2_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 3 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame3_1),'ROI',objectRegion3_1, 'MinQuality', 0.001);
tracker = vision.PointTracker('MaxBidirectionalError',5);
initialize(tracker,points.Location,objectFrame3_1);

i = 0;
while ~isDone(videoFileReader3_1)
    i = i+1;
    frame = videoFileReader3_1();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x3_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y3_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer3_1(out);
    pause(0.01)
end

for i = 1:size3_1(4)-1
    imshow(vidFrames3_2(:, :, :, i))
    hold on
    plot(x3_1(i), y3_1(i), 'r*')
    hold off
    pause(0.01)
end


%% Aligning video feeds

frame_rate = 30;

t1 = (1:1:(size1_1(4)-1))/frame_rate;
t2 = (1:1:(size2_1(4)-1))/frame_rate;
t3 = (1:1:(size3_1(4)-1))/frame_rate;

figure(1)
plot(t1, x1_1, t1, y1_1)
legend("x", "y")

figure(2)
plot(t2, x2_1, t2, y2_1)
legend("x", "y")


figure(3)
plot(t3, x3_1, t3, y3_1)
legend("x", "y")

x1_1_truncated = x1_1(11:310);
y1_1_truncated = y1_1(11:310);
t1_truncated = t1(11:310) - t1(11);

x2_1_truncated = x2_1(2:301);
y2_1_truncated = y2_1(2:301);
t2_truncated = t2(2:301) - t2(2);

x3_1_truncated = x3_1(16:315);
y3_1_truncated = y3_1(16:315);
t3_truncated = t3(16:315) - t3(16);

figure(4)
plot(t1_truncated, x1_1_truncated, t1_truncated, y1_1_truncated)
legend("x", "y")

figure(5)
plot(t2_truncated, x2_1_truncated, t2_truncated, y2_1_truncated)
legend("x", "y")

figure(6)
plot(t3_truncated, x3_1_truncated, t3_truncated, y3_1_truncated)
legend("x", "y")

%% Computing SVD

X = [x1_1_truncated; y1_1_truncated; x2_1_truncated;...
    y2_1_truncated; x3_1_truncated; y3_1_truncated];

[U,S,V] = svd(X);

figure(7)
subplot(2,2,1)
plot(t2_truncated, x2_1_truncated, t2_truncated, y2_1_truncated)
ylim([240 400])
for j=1:3
  ff=U(:,1:j)*S(1:j,1:j)*V(:,1:j)'; % modal projections 
  subplot(2,2,j+1)
  plot(t1_truncated, ff(3,:), t1_truncated, ff(4,:))
  ylim([240 400])
end

figure(9)
sig=diag(S);

subplot(2,2,1), plot(sig,'ko','Linewidth',[1.5])
% axis([0 25 0 50])
% set(gca,'Fontsize',[13],'Xtick',[0 5 10 15 20 25]) 
% text(20,40,'(a)','Fontsize',[13])

subplot(2,2,2), semilogy(sig,'ko','Linewidth',[1.5])
% axis([0 25 10^(-18) 10^(5)])
% set(gca,'Fontsize',[13],'Ytick',[10^(-15) 10^(-10) 10^(-5) 10^0 10^5],...
%    'Xtick',[0 5 10 15 20 25]); 
% text(20,10^0,'(b)','Fontsize',[13])

subplot(2,1,2) 
plot(t1_truncated,V(:,1),'k',t2_truncated,V(:,2),'k--',t3_truncated,V(:,3),'k:','Linewidth',[2]) 
% set(gca,'Fontsize',[13])
legend('mode 1','mode 2','mode 3','Location','NorthWest') 
% text(0.8,0.35,'(c)','Fontsize',[13])

