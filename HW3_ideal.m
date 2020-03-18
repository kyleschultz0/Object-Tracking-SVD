clc; clear all; close all

%% Ideal case
%% Create .avi from given .mat file to use with CV toolbox

load("cam1_1.mat");
videoFWriter = vision.VideoFileWriter('cam1_1.avi');
size1_1 = size(vidFrames1_1);

for i=1:size1_1(4)
  videoFrame = vidFrames1_1(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam2_1.mat");
videoFWriter = vision.VideoFileWriter('cam2_1.avi');
size2_1 = size(vidFrames2_1);

for i=1:size2_1(4)
  videoFrame = vidFrames2_1(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam3_1.mat");
videoFWriter = vision.VideoFileWriter('cam3_1.avi');
size3_1 = size(vidFrames3_1);

for i=1:size3_1(4)
  videoFrame = vidFrames3_1(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

%%  Create object region

videoFileReader1_1 = vision.VideoFileReader('cam1_1.avi');
videoPlayer1_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame1_1 = videoFileReader1_1();
figure; imshow(objectFrame1_1);
objectRegion1_1=round(getPosition(imrect));

videoFileReader2_1 = vision.VideoFileReader('cam2_1.avi');
videoPlayer2_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame2_1 = videoFileReader2_1();
figure; imshow(objectFrame2_1);
objectRegion2_1=round(getPosition(imrect));

videoFileReader3_1 = vision.VideoFileReader('cam3_1.avi');
videoPlayer3_1 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame3_1 = videoFileReader3_1();
figure; imshow(objectFrame3_1);
objectRegion3_1=round(getPosition(imrect));




%% Track Points

% Camera 1 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame1_1),'ROI',objectRegion1_1);
tracker = vision.PointTracker('MaxBidirectionalError',1);
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
    imshow(vidFrames1_1(:, :, :, i))
    hold on
    plot(x1_1(i), y1_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 2 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame2_1),'ROI',objectRegion2_1);
tracker = vision.PointTracker('MaxBidirectionalError',1);
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
    imshow(vidFrames2_1(:, :, :, i))
    hold on
    plot(x2_1(i), y2_1(i), 'r*')
    hold off
    pause(0.01)
end

% Camera 3 -------------------------------------------
points = detectMinEigenFeatures(rgb2gray(objectFrame3_1),'ROI',objectRegion3_1);
tracker = vision.PointTracker('MaxBidirectionalError',1);
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
    imshow(vidFrames3_1(:, :, :, i))
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

x1_1_truncated = x1_1(10:225) - mean(x1_1(10:225));
y1_1_truncated = y1_1(10:225) - mean(y1_1(10:225));
t1_truncated = t1(10:225) - t1(10);

x2_1_truncated = x2_1(18:233) - mean(x2_1(18:233));
y2_1_truncated = y2_1(18:233) - mean(y2_1(18:233));
t2_truncated = t2(18:233) - t2(18);

x3_1_truncated = x3_1(8:223) - mean(x3_1(8:223));
y3_1_truncated = y3_1(8:223) - mean(y3_1(8:223));
t3_truncated = t3(8:223) - t3(8);

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
plot(t1_truncated, x1_1_truncated, t1_truncated, y1_1_truncated)
ylim([240 400])
for j=1:3
  ff=U(:,1:j)*S(1:j,1:j)*V(:,1:j)'; % modal projections 
  subplot(2,2,j+1)
  plot(t1_truncated, ff(1,:), t1_truncated, ff(2,:))
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

%% SVD Directions

c = 200;
figure(10)
imshow(vidFrames2_1(:, :, :, 100))
hold on
for i = 100:120
    plot(x2_1(i), y2_1(i), 'r*')
end

plot([x2_1(110), x2_1(110) + c*U(1, 4)], [y2_1(110), y2_1(110) + c*U(1, 3)], 'linewidth', 3)
hold off

title("Paint Can Vertical Displacement")




