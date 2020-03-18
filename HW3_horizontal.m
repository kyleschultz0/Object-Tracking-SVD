clc; clear all; close all

%% horizontal case
%% Create .avi from given .mat file to use with CV toolbox

load("cam1_3.mat");
videoFWriter = vision.VideoFileWriter('cam1_3.avi');
size1_3 = size(vidFrames1_3);

for i=1:size1_3(4)
  videoFrame = vidFrames1_3(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam2_3.mat");
videoFWriter = vision.VideoFileWriter('cam2_3.avi');
size2_3 = size(vidFrames2_3);

for i=1:size2_3(4)
  videoFrame = vidFrames2_3(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam3_3.mat");
videoFWriter = vision.VideoFileWriter('cam3_3.avi');
size3_3 = size(vidFrames3_3);

for i=1:size3_3(4)
  videoFrame = vidFrames3_3(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);
%%  Create object region

videoFileReader1_3 = vision.VideoFileReader('cam1_3.avi');
videoPlayer1_3 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame1_3 = videoFileReader1_3();
figure; imshow(objectFrame1_3);
objectRegion1_3=round(getPosition(imrect));

videoFileReader2_3 = vision.VideoFileReader('cam2_3.avi');
videoPlayer2_3 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame2_3 = videoFileReader2_3();
figure; imshow(objectFrame2_3);
objectRegion2_3=round(getPosition(imrect));

videoFileReader3_3 = vision.VideoFileReader('cam3_3.avi');
videoPlayer3_3 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame3_3 = videoFileReader3_3();
figure; imshow(objectFrame3_3);
objectRegion3_3=round(getPosition(imrect));




%% Track Points

% Camera 1 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame1_3),'ROI',objectRegion1_3, 'MinQuality', 0.1, 'FilterSize', 91);
tracker = vision.PointTracker('MaxBidirectionalError',10);
initialize(tracker,points.Location,objectFrame1_3);

i = 0;
while ~isDone(videoFileReader1_3)
    i = i+1;
    frame = videoFileReader1_3();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x1_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y1_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer1_3(out);
    pause(0.01)
end

for i = 1:size1_3(4)-1
    imshow(vidFrames1_3(:, :, :, i))
    hold on
    plot(x1_1(i), y1_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 2 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame2_3),'ROI',objectRegion2_3, 'MinQuality', 1, 'FilterSize', 91);
tracker = vision.PointTracker('MaxBidirectionalError',5);
initialize(tracker,points.Location,objectFrame2_3);

i = 0;
while ~isDone(videoFileReader2_3)
    i = i+1;
    frame = videoFileReader2_3();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x2_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y2_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer2_3(out);
    pause(0.01)
end

for i = 1:size2_3(4)-1
    imshow(vidFrames2_3(:, :, :, i))
    hold on
    plot(x2_1(i), y2_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 3 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame3_3),'ROI',objectRegion3_3, 'MinQuality', 0.3, 'FilterSize', 91);
tracker = vision.PointTracker('MaxBidirectionalError',10);
initialize(tracker,points.Location,objectFrame3_3);

i = 0;
while ~isDone(videoFileReader3_3)
    i = i+1;
    frame = videoFileReader3_3();
    [points,validity] = tracker(frame);
    points = points(validity, :);
    x3_1(i) = round(sum(points(:,1))/length(points(:,1)));
    y3_1(i) = round(sum(points(:,2))/length(points(:,2)));
    [points,validity] = tracker(frame);
    out = insertMarker(frame,points(validity, :),'+');
    videoPlayer3_3(out);
    pause(0.01)
end

for i = 1:size3_3(4)-1
    imshow(vidFrames3_3(:, :, :, i))
    hold on
    plot(x3_1(i), y3_1(i), 'r*')
    hold off
    pause(0.01)
end

%% Aligning video feeds

frame_rate = 30;

t1 = (1:1:(size1_3(4)-1))/frame_rate;
t2 = (1:1:(size2_3(4)-1))/frame_rate;
t3 = (1:1:(size3_3(4)-1))/frame_rate;

figure(1)
plot(t1, x1_1, t1, y1_1)
legend("x", "y")

figure(2)
plot(t2, x2_1, t2, y2_1)
legend("x", "y")


figure(3)
plot(t3, x3_1, t3, y3_1)
legend("x", "y")

x1_1_truncated = x1_1(16:236) - mean(x1_1(16:236));
y1_1_truncated = y1_1(16:236) - mean(y1_1(16:236));
t1_truncated = t1(16:236) - t1(16);

x2_1_truncated = x2_1(21:241) - mean(x2_1(21:241));
y2_1_truncated = y2_1(21:241) - mean(y2_1(21:241));
t2_truncated = t2(21:241) - t2(21);

x3_1_truncated = x3_1(11:231) - mean(x3_1(11:231));
y3_1_truncated = y3_1(11:231) - mean(y3_1(11:231));
t3_truncated = t3(11:231) - t3(11);

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

X = [x1_1_truncated/max(x1_1_truncated); y1_1_truncated/max(y1_1_truncated); x2_1_truncated/max(x2_1_truncated);...
    y2_1_truncated/max(y2_1_truncated); x3_1_truncated/max(x3_1_truncated); y3_1_truncated/max(x3_1_truncated)];

[U,S,V] = svd(X);

figure(7)
subplot(2,2,1)
plot(t2_truncated, x2_1_truncated, t2_truncated, y2_1_truncated)
ylim([200 400])
for j=1:3
  ff=U(:,1:j)*S(1:j,1:j)*V(:,1:j)'; % modal projections 
  subplot(2,2,j+1)
  plot(t1_truncated, ff(3,:), t1_truncated, ff(4,:))
  ylim([200 400])
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

%%

c = 100;
figure(10)
imshow(vidFrames2_3(:, :, :, 100))
hold on
for i = 20:120
    plot(x2_1(i), y2_1(i), 'r*')
end

plot([x2_1(100), x2_1(100) + c*U(3, 1)], [y2_1(100), y2_1(100) + c*U(4, 1)], 'linewidth', 3)
plot([x2_1(100), x2_1(100) + c*U(3, 2)], [y2_1(100), y2_1(100) + c*U(4, 2)], 'linewidth', 3)
hold off

title("Paint Can Vertical and Horizontal Displacement")

