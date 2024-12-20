function [xoffset,yoffset, xpeak, ypeak] = ncc(img, t)

    img_k = imread(img);
    img_k_gray = rgb2gray(img_k);    
    score_map = normxcorr2(t, img_k_gray);
    [ypeak, xpeak] = find(score_map == max(score_map(:)));  
    xoffset = xpeak - size(t,2);
    yoffset = ypeak - size(t,1);
    
    end