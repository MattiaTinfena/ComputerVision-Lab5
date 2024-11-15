%% Lab5
clc;clear;close all;
%% NCC-based segmantation

%Load images
img1 = imread('ur_c_s_03a_01_L_0376.png');
img1 = rgb2gray(img1);

%Template definition
T=img1(350:430, 680:780);
T11 = img1(357:417, 544:651);
T12=img1(390:400, 575:595);
T13 = img1(300:450, 500:680); 

% Create a vector t with T11, T12, and T13 inside
t = {T11, T12, T13};
times = {0, 0, 0};
timeNCC = {0, 0, 0, 0, 0, 0};
timeCBS = {0, 0, 0, 0, 0, 0};

figure 
subplot(2, 3, 2)
imagesc(T);
colormap gray;
title('Template 1');

for k = 1:3
    subplot(2, 3, k + 3)
    imagesc(t{k});
    colormap gray;
    title(['Template 2, size ', num2str(k)]);
end

image_files = {"ur_c_s_03a_01_L_0376.png", "ur_c_s_03a_01_L_0377.png", "ur_c_s_03a_01_L_0378.png","ur_c_s_03a_01_L_0379.png", "ur_c_s_03a_01_L_0380.png", "ur_c_s_03a_01_L_0381.png"};

%Red car

figure;
for k = 1:length(image_files)

    tic;
    [xoffset, yoffset, xpeak, ypeak] = ncc(image_files{k}, T);
    timeNCC{k} = toc;
    
    tic;
    [centroid, bounding_box] = cbs(image_files{k});
    timeCBS{k} = toc;
   
    subplot(2, 3, k)  
    imshow(image_files{k});
    hold on;    
    rectangle('Position', [xoffset , yoffset , size(T,2), size(T,1)],'EdgeColor', 'b', 'LineWidth', 2);    
    plot(xpeak - size(T,2)/2, ypeak - size(T,1)/2, '*b', 'LineWidth', 2);
    plot(centroid(1), centroid(2),'*g')
    rectangle('Position',bounding_box,'EdgeColor','g', 'LineWidth', 2)
    title(['Detected Position in Image ', num2str(k)]);
    hold off;
end
% sgtitle('Normalized Cross-Correlation results to detect the red car');
sgtitle('Comparison between NCC and CBS to detect the red car');
NCCavg = mean(cell2mat(timeNCC));
CBSavg = mean(cell2mat(timeCBS));

disp(['NCC average execution time is: ', num2str(NCCavg), ' seconds']);
disp(['CBS average execution time is: ', num2str(CBSavg), ' seconds']);

%Dark car
for i = 1:3
    tic;
    figure;
    for k = 1:length(image_files)
        [xoffset, yoffset, xpeak, ypeak] = ncc(image_files{k}, t{i});
        subplot(2, 3, k)  
        imshow(image_files{k});
        hold on;    
        rectangle('Position', [xoffset , yoffset , size(t{i},2), size(t{i},1)],'EdgeColor', 'r', 'LineWidth', 2);    
        plot(xpeak- (size(t{i},2) / 2), ypeak - (size(t{i},1) / 2), 'r*', 'MarkerSize', 4, 'LineWidth', 4);
        title(['Detected Position in Image ', num2str(k)]);
        hold off;
    end
    times{i} = toc;
    if i == 1
        sgtitle('Normalized Cross-Correlation results to detect the dark car (Normal Window)');
    elseif i == 2
        sgtitle('Normalized Cross-Correlation results to detect the dark car (Small Window)');
    else
        sgtitle('Normalized Cross-Correlation results to detect the dark car (Large Window)');
    end
    disp(['execution ', num2str(i), ' time: ',num2str(times{i}), ' seconds']);
end

%% Harris corner detection
img = double(imread("i235.png"));

dx=[1 0 -1; 2 0 -2; 1 0 -1];
dy=[1 2 1; 0  0  0; -1 -2 -1];
Ix=conv2(img, dx, 'same');
Iy=conv2(img, dy, 'same');
Ix2=Ix.*Ix; Iy2=Iy.*Iy; Ixy=Ix.*Iy;

g = fspecial('gaussian', 9, 1.2);

figure,
subplot(1, 3, 1)
imagesc(Ix),colormap gray, title('partial derative Ix')
subplot(1, 3, 2)
imagesc(Iy),colormap gray, title('partial derative Iy')
subplot(1, 3, 3)
imagesc(g),colormap gray, title('Gaussian filter')

Sx2=conv2(Ix2,g,'same'); Sy2=conv2(Iy2,g,'same'); Sxy=conv2(Ixy,g,'same');

%features detection
[rr,cc]=size(Sx2);
edge_reg=zeros(rr,cc); corner_reg=zeros(rr,cc); flat_reg=zeros(rr,cc);
R_map=zeros(rr,cc);
k=0.05;

for ii=1:rr
    for jj=1:cc
        %define at each pixel x,y the matrix
        M=[Sx2(ii,jj),Sxy(ii,jj);Sxy(ii,jj),Sy2(ii,jj)];
        %compute the response of the detector at each pixel
        R=det(M) - k*(trace(M).^2);
        R_map(ii,jj)=R;
    end
end

figure
imagesc(R_map);
title('R score map')

M = max(R_map(:));
threshold = 0.3 * M;

corner_reg = R_map > threshold;
figure,imagesc(corner_reg.*img),colormap gray,title('corner regions')

figure,imagesc(img),colormap gray,title('detected objects')
prop = regionprops(corner_reg, 'Centroid');
centroids = cat(1, prop.Centroid);
hold on
plot(centroids(:,1),centroids(:,2),'r*')
hold off