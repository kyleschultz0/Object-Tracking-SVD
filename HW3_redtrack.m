clc; clear all; close all

%% Rotating case
%% Create .avi for CV

load("cam1_4.mat");
vidFrames1_4 = vidFrames1_4(179:179+218, 321:321+165, :, :);
videoFWriter = vision.VideoFileWriter('cam1_4.avi');
size1_4 = size(vidFrames1_4);
videoFileReader1_4 = vision.VideoFileReader('cam1_4.avi');

for i=1:size1_4(4)
  videoFrame = vidFrames1_4(:, :, :, i);
  videoFWriter(videoFrame);
end

load("cam2_4.mat");
vidFrames2_4 = vidFrames2_4(90:90+300, 160:160+290, :, :);
videoFWriter = vision.VideoFileWriter('cam2_4.avi');
size2_4 = size(vidFrames2_4);
videoFileReader2_4 = vision.VideoFileReader('cam2_4.avi');


for i=1:size2_4(4)
  videoFrame = vidFrames2_4(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

load("cam3_4.mat");
vidFrames3_4 = vidFrames3_4(139:139+136, 300:319+160, :, :);
videoFWriter = vision.VideoFileWriter('cam3_4.avi');
size3_4 = size(vidFrames3_4);
videoFileReader3_4 = vision.VideoFileReader('cam3_4.avi');


for i=1:size3_4(4)
  videoFrame = vidFrames3_4(:, :, :, i);
  videoFWriter(videoFrame);
end
release(videoFWriter);

%% Get object region

release(videoFWriter);
videoPlayer1_4 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame1_4 = videoFileReader1_4();
figure; imshow(objectFrame1_4);
objectRegion1_4=round(getPosition(imrect));

release(videoFWriter);
videoPlayer2_4 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame2_4 = videoFileReader2_4();
figure; imshow(objectFrame2_4);
objectRegion2_4=round(getPosition(imrect));

release(videoFWriter);
videoPlayer3_4 = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame3_4 = videoFileReader3_4();
figure; imshow(objectFrame3_4);
objectRegion3_4=round(getPosition(imrect));

%% Find matching RGB values from region
figure(1)

% Camera 1 ----------------------------------------
RGB_rect1 = vidFrames1_4(objectRegion1_4(2):objectRegion1_4(2)+objectRegion1_4(4),...
    objectRegion1_4(1):objectRegion1_4(1)+objectRegion1_4(3), :, 1);

RGB_rect1 = double(RGB_rect1);

RGB_rect_size1 = size(RGB_rect1);
length_rect1 = RGB_rect_size1(1)*RGB_rect_size1(2);

R_mean1 = sum(RGB_rect1(:, :, 1), 'all')/length_rect1;
R_std1 = 0.45*sqrt(var(RGB_rect1(:, :, 1),0,'all'));
R_min1 = R_mean1 - R_std1; R_max1 = R_mean1 + R_std1; 

G_mean1 = sum(RGB_rect1(:, :, 2), 'all')/length_rect1;
G_std1 = 0.45*sqrt(var(RGB_rect1(:, :, 2),0,'all'));
G_min1 = G_mean1 - G_std1; G_max1 = G_mean1 + G_std1; 

B_mean1 = sum(RGB_rect1(:, :, 3), 'all')/length_rect1;
B_std1 = 0.45*sqrt(var(RGB_rect1(:, :, 3),0,'all'));
B_min1 = B_mean1 - B_std1; B_max1 = B_mean1 + B_std1; 


for i = 1:size1_4(4)
    num = 0;
    for j = 1:size1_4(1)
        for k = 1:size1_4(2)
            if (vidFrames1_4(j, k, 1, i)>R_min1) && (vidFrames1_4(j, k, 1, i)<R_max1)...
                    && (vidFrames1_4(j, k, 2, i)>G_min1) && (vidFrames1_4(j, k, 2, i)<G_max1)...
                    && (vidFrames1_4(j, k, 3, i)>B_min1) && (vidFrames1_4(j, k, 3, i)<B_max1)
                num = num+1;
                points1(num,1,i) = k;
                points1(num,2,i) = j;
            end
        end
    end
    if num == 0
        points1(1,1,i) = 1;
        points1(1,2,i) = 1;
    end  
end

for i = 1:size1_4(4)
    imshow(vidFrames1_4(:, :, :, i))
    hold on
    scatter(points1(:, 1, i), points1(:,2,i), 'r*')
    hold off
    pause(0.01)
end

% Camera 2 ----------------------------------------
RGB_rect2 = vidFrames2_4(objectRegion2_4(2):objectRegion2_4(2)+objectRegion2_4(4),...
    objectRegion2_4(1):objectRegion2_4(1)+objectRegion2_4(3), :, 1);

RGB_rect2 = double(RGB_rect2);

RGB_rect_size2 = size(RGB_rect2);
length_rect2 = RGB_rect_size2(1)*RGB_rect_size2(2);

R_mean2 = sum(RGB_rect2(:, :, 1), 'all')/length_rect2;
R_std2 = 0.45*sqrt(var(RGB_rect2(:, :, 1),0,'all'));
R_min2 = R_mean2 - R_std2; R_max = R_mean2 + R_std2; 

G_mean2 = sum(RGB_rect2(:, :, 2), 'all')/length_rect2;
G_std2 = 0.45*sqrt(var(RGB_rect2(:, :, 2),0,'all'));
G_min2 = G_mean2 - G_std2; G_max = G_mean2 + G_std2; 

B_mean2 = sum(RGB_rect2(:, :, 3), 'all')/length_rect2;
B_std2 = 0.45*sqrt(var(RGB_rect2(:, :, 3),0,'all'));
B_min2 = B_mean2 - B_std2; B_max = B_mean2 + B_std2; 


for i = 1:size2_4(4)
    num = 0;
    for j = 1:size2_4(1)
        for k = 1:size2_4(2)
            if (vidFrames2_4(j, k, 1, i)>R_min2) && (vidFrames2_4(j, k, 1, i)<R_max)...
                    && (vidFrames2_4(j, k, 2, i)>G_min2) && (vidFrames2_4(j, k, 2, i)<G_max)...
                    && (vidFrames2_4(j, k, 3, i)>B_min2) && (vidFrames2_4(j, k, 3, i)<B_max)
                num = num+1;
                points2(num,1,i) = k;
                points2(num,2,i) = j;
            end
        end
    end
    if num == 0
        points2(1,1,i) = 1;
        points2(1,2,i) = 1;
    end
end

for i = 1:size2_4(4)
    imshow(vidFrames2_4(:, :, :, i))
    hold on
    scatter(points2(:, 1, i), points2(:,2,i), 'r*')
    hold off
    pause(0.01)
end
    
% Camera 3 ----------------------------------------

RGB_rect3 = vidFrames3_4(objectRegion3_4(2):objectRegion3_4(2)+objectRegion3_4(4),...
    objectRegion3_4(1):objectRegion3_4(1)+objectRegion3_4(3), :, 1);

RGB_rect3 = double(RGB_rect3);

RGB_rect_size3 = size(RGB_rect3);
length_rect3 = RGB_rect_size3(1)*RGB_rect_size3(2);

R_mean3 = sum(RGB_rect3(:, :, 1), 'all')/length_rect3;
R_std3 = 1.2*sqrt(var(RGB_rect3(:, :, 1),0,'all'));
R_min3 = R_mean3 - R_std3; R_max3 = R_mean3 + R_std3; 

G_mean3 = sum(RGB_rect3(:, :, 2), 'all')/length_rect3;
G_std3 = 1.2*sqrt(var(RGB_rect3(:, :, 2),0,'all'));
G_min3 = G_mean3 - G_std3; G_max3 = G_mean3 + G_std3; 

B_mean3 = sum(RGB_rect3(:, :, 3), 'all')/length_rect3;
B_std3 = 1.2*sqrt(var(RGB_rect3(:, :, 3),0,'all'));
B_min3 = B_mean3 - B_std3; B_max3 = B_mean3 + B_std3; 


for i = 1:size3_4(4)
    num = 0;
    for j = 1:size3_4(1)
        for k = 1:size3_4(2)
            if (vidFrames3_4(j, k, 1, i)>R_min3) && (vidFrames3_4(j, k, 1, i)<R_max3)...
                    && (vidFrames3_4(j, k, 2, i)>G_min3) && (vidFrames3_4(j, k, 2, i)<G_max3)...
                    && (vidFrames3_4(j, k, 3, i)>B_min3) && (vidFrames3_4(j, k, 3, i)<B_max3)
                num = num+1;
                points3(num,1,i) = k;
                points3(num,2,i) = j;
            end
        end
    end
    if num == 0
        points3(1,1,i) = 1;
        points3(1,2,i) = 1;
    end
end

for i = 1:size3_4(4)
    imshow(vidFrames3_4(:, :, :, i))
    hold on
    scatter(points3(:, 1, i), points3(:,2,i), 'r*')
    hold off
    pause(0.01)
end

%% Plotting 

frame_rate = 30;

t1 = (1:1:(size1_4(4)))/frame_rate;
t2 = (1:1:(size2_4(4)))/frame_rate;
t3 = (1:1:(size3_4(4)))/frame_rate;

for i = 1:size1_4(4)
    x1(i) = sum(points1(:,1,i))/sum(points1(:,1,i)~=0);
    y1(i) = sum(points1(:,2,i))/sum(points1(:,2,i)~=0);
end
    
for i = 1:size2_4(4)
    x2(i) = sum(points2(:,1,i))/sum(points2(:,1,i)~=0);
    y2(i) = sum(points2(:,2,i))/sum(points2(:,2,i)~=0);
end

for i = 1:size3_4(4)
    x3(i) = sum(points3(:,1,i))/sum(points3(:,1,i)~=0);
    y3(i) = sum(points3(:,2,i))/sum(points3(:,2,i)~=0);
end

figure(1)
plot(t1, x1, t1, y1)

figure(2)
plot(t2, x2, t2, y2)

figure(3)
plot(t3, x3, t3, y3)

%% Filter

figure(5)
x1f = lowpass(x1,2,frame_rate);
y1f = lowpass(y1,2,frame_rate);
plot(t1, x1f, t1, x1)
legend("Filtered", "Raw")

figure(6)
x2f = lowpass(x2,2,frame_rate);
y2f = lowpass(y2,2,frame_rate);
plot(t2, x2f, t2, x2)
legend("Filtered", "Raw")

figure(7)
x3f = lowpass(x3,2,frame_rate);
y3f = lowpass(y3,2,frame_rate);
plot(t3, x3f, t3, x3)
legend("Filtered", "Raw")
    
    
%% SVD




x1_1_truncated = x1f(33:380) - mean(x1f(33:380));
y1_1_truncated = y1f(33:380) - mean(y1f(33:380));
t1_truncated = t1(33:380) - t1(33);

x2_1_truncated = x2f(20:367) - mean(x2f(20:367));
y2_1_truncated = y2f(20:367) - mean(y2f(20:367));
t2_truncated = t2(20:367) - t2(20);

x3_1_truncated = x3f(32:379) - mean(x3f(32:379));
y3_1_truncated = y3f(32:379) - mean(y3f(32:379));
t3_truncated = t3(32:379) - t3(32);


X = [x1_1_truncated; y1_1_truncated; x2_1_truncated;...
    y2_1_truncated; x3_1_truncated; y3_1_truncated];

[U,S,V] = svd(X);

figure(8)
subplot(2,2,1)
plot(t2_truncated, x2_1_truncated, t2_truncated, y2_1_truncated)
for j=1:3
  ff=U(:,1:j)*S(1:j,1:j)*V(:,1:j)'; % modal projections 
  subplot(2,2,j+1)
  plot(t1_truncated, ff(3,:), t1_truncated, ff(4,:))
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
imshow(vidFrames1_4(:, :, :, 100))
hold on
for i = 100:120
    plot(x1f(i), y1f(i), 'r*')
end

plot([x1f(110), x1f(110) + c*U(1, 1)], [y1f(110), y1f(110) + c*U(2, 2)], 'linewidth', 3)
plot([x1f(110), x1f(110) + c*U(1, 2)], [y1f(110), y1f(110) + c*U(2, 2)], 'linewidth', 3)
plot([x1f(110), x1f(110) + c*U(1, 3)], [y1f(110), y1f(110) + c*U(2, 3)], 'linewidth', 3)

hold off

title("Paint Can Vertical Displacement")

    
                    
                   



