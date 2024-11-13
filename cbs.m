function [centroid1, bounding_box1] = cbs(img)

    img_k = imread(img);
    img_hsv = rgb2hsv(img_k);

    images_h = img_hsv(:,:,1);   % Canale H
    mask1 = (images_h > (0.97)) & (images_h < (1));
    images_seg1 = bwlabel(mask1);

    prop1 = regionprops(images_seg1, 'Area','Centroid','BoundingBox');

    [~, largest_idx1] = max([prop1.Area]);
    centroid1 = prop1(largest_idx1).Centroid;
    bounding_box1 = prop1(largest_idx1).BoundingBox;
end