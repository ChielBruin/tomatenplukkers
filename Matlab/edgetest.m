img_raw  = imread('~/Bureaublad/tmp/color/IMG_20160923_121235.jpg');
img_raw = img_raw(:, : ,2);
%imagesc(img_raw);
img = imgaussfilt(img_raw);
imshow(img);
BW1 = edge(img,'sobel', [], 'vertical');
figure;
imshow(BW1);