%% Lab5

%Load images
img1 = imread('ur_c_s_03a_01_L_0376.png');
img1 = rgb2gray(img1);


%Template definition
T=img1(350:430, 680:780);


image_files = {"ur_c_s_03a_01_L_0376.png", "ur_c_s_03a_01_L_0377.png", "ur_c_s_03a_01_L_0378.png","ur_c_s_03a_01_L_0379.png", "ur_c_s_03a_01_L_0380.png", "ur_c_s_03a_01_L_0381.png"};
figure;
for k = 1:length(image_files)    
    img_k = imread(image_files{k});
    img_k_gray = rgb2gray(img_k);    
    score_map = normxcorr2(T, img_k_gray);   
    % imshow(score_map)
    [ypeak, xpeak] = find(score_map == max(score_map(:)), 1);    
    yoffSet = ypeak - size(T, 1);
    xoffSet = xpeak - size(T, 2);    
    subplot(2, 3, k)  
    imshow(img_k);
    hold on;    
    rectangle('Position', [xoffSet, yoffSet, size(T,2), size(T,1)],'EdgeColor', 'r', 'LineWidth', 2);    
    plot(xoffSet, yoffSet, 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    title(['Detected Position in Image ', num2str(k)]);
    hold off;
end



% 
% 
% TM=filter2(T,img1,'same');%two-dimensional correlation
% % figure,imagesc(TM),colormap gray
% 
% T1=T-mean2(T);
% 
% TM1=filter2(T1,img1,'same');%two-dimensional correlation
% % figure,imagesc(TM1),colormap gray
% 
% C = normxcorr2(T, img1);%Normalized 2-D cross-correlation
% figure,imagesc(C),colormap gray
% 
% [ypeak, xpeak] = find(C == max(C(:)));
% yoffSet = ypeak - size(T, 1);
% xoffSet = xpeak - size(T, 2);
% 
% figure;
% imshow(img1);
% 
% hold on;
% rectangle('Position', [xoffSet, yoffSet, size(T,2), size(T,1)],'EdgeColor', 'r', 'LineWidth', 2);
% hold off;