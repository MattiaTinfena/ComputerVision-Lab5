%% Lab5

%% NCC-based segmantation

%Load images
img1 = imread('ur_c_s_03a_01_L_0376.png');
img1 = rgb2gray(img1);

%Template definition
T=img1(350:430, 680:780);
T1 = img1(357:417, 544:651);

image_files = {"ur_c_s_03a_01_L_0376.png", "ur_c_s_03a_01_L_0377.png", "ur_c_s_03a_01_L_0378.png","ur_c_s_03a_01_L_0379.png", "ur_c_s_03a_01_L_0380.png", "ur_c_s_03a_01_L_0381.png"};

figure;
for k = 1:length(image_files)  
    
    img_k = imread(image_files{k});
    img_k_gray = rgb2gray(img_k);    
    score_map = normxcorr2(T, img_k_gray);

    [xsm, ysm] = size(score_map);
    [xim, yim, channels] = size(img_k);
    xdiff = (xsm - xim)/2;
    ydiff = (ysm - yim)/2;
    [ypeak, xpeak] = find(score_map == max(score_map(:)), 1);  
    xoffset = xpeak - xdiff;
    yoffset = ypeak - ydiff;
    
    %print
    subplot(2, 3, k)  
    imshow(img_k);
    hold on;    
    rectangle('Position', [xoffset - (size(T,2) / 2), yoffset - (size(T,1) / 2), size(T,2), size(T,1)],'EdgeColor', 'r', 'LineWidth', 2);    
    plot(xoffset, yoffset, 'ro', 'MarkerSize', 4, 'LineWidth', 4);
    title(['Detected Position in Image ', num2str(k)]);
    hold off;
end


figure;
for k = 1:length(image_files)  
    
    img_k = imread(image_files{k});
    img_k_gray = rgb2gray(img_k);    
    score_map = normxcorr2(T1, img_k_gray);

    [xsm, ysm] = size(score_map);
    [xim, yim, channels] = size(img_k);
    xdiff = (xsm - xim)/2;
    ydiff = (ysm - yim)/2;
    [ypeak, xpeak] = find(score_map == max(score_map(:)), 1);  
    xoffset = xpeak - xdiff;
    yoffset = ypeak - ydiff;
    
    %print
    subplot(2, 3, k)  
    imshow(img_k);
    hold on;    
    rectangle('Position', [xoffset - (size(T,2) / 2), yoffset - (size(T,1) / 2), size(T,2), size(T,1)],'EdgeColor', 'r', 'LineWidth', 2);    
    plot(xoffset, yoffset, 'ro', 'MarkerSize', 4, 'LineWidth', 4);
    title(['Detected Position in Image ', num2str(k)]);
    hold off;
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
imagesc(Ix),colormap gray,title('partial derative Ix')
subplot(1, 3, 2)
imagesc(Iy),colormap gray,title('partial derative Iy')
subplot(1, 3, 3)
imagesc(g),colormap gray,title('Gaussian filter')

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

M = max(R_map(:));
threshold = 0.3 * M;

corner_reg = R_map > threshold;
figure,imagesc(corner_reg.*img),colormap gray,title('corner regions')

figure,imagesc(img),colormap gray,title('detected object')
prop = regionprops(corner_reg, 'Centroid');
centroids = cat(1, prop.Centroid);
hold on
plot(centroids(:,1),centroids(:,2),'r*')
hold off