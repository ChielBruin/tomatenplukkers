function [goodMu, goodSigma, badMu, badSigma] = findProbValues(good_image, bad_image)
    [goodMu, goodSigma] = findValues(good_image);    
    [badMu, badSigma] = findValues(bad_image);
end

function [mu, sigma] = findValues(imagePath) 
    image_raw = double(imread(imagePath))/255;
    image_r = image_raw(:, : ,1);
    image_r = image_r(image_r < 1);
    image_g = image_raw(:, : ,2);
    image_g = image_g(image_g < 1);
	image_b = image_raw(:, : ,3);
    image_b = image_b(image_b < 1);
    
    mu    = [mean(image_r), mean(image_g), mean(image_b)];
    sigma = [std(image_r), std(image_g), std(image_b)];
end