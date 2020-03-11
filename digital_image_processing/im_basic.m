img = imread('dipum_images_ch02/Fig0204(a)(bubbles-q-100jpg).tif');
imshow(img, [ ]);
imwrite(img, 'bubbles_25.jpg', 'quality', 25);
imfinfo('bubbles_25.jpg')
