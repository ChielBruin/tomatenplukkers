%% Process the specified image to recognize cucumbers.
%  A preview can be shown with all the intermediate images.
function [image] = imageProc(imagePath, preview)    
    goodMu = [0.5410, 0.5467, 0.5282];
    goodSigma = [0.1844, 0.1934, 0.1918];
    
    badMu = [0.4314, 0.4523, 0.4665];
    badSigma = [0.1440, 0.1421, 0.1427];
    
    image = customImageProc(imagePath, goodMu, goodSigma, badMu, badSigma, preview);
end

%% Process the specified image using custom mu's and sigmas.
% using his method directly is discouraged, please use the imageProc
% function.
function [image_drule] = customImageProc(imagePath, goodMu, goodSigma, badMu, badSigma, preview)
    image_raw = double(imread(imagePath))/255;
    image_r = image_raw(:, : ,1);
    image_g = image_raw(:, : ,2);
	image_b = image_raw(:, : ,3);
    
    image_totals = image_r + image_g + image_b;
    
    image_pure_r = image_r./image_totals;
	image_pure_g = image_g./image_totals;
	image_pure_b = image_b./image_totals;
    
    image_pure = cat(3, image_pure_r, image_pure_g, image_pure_b);
    
    % Apply the decision rule in parallel
    parfor i = 1:3
        image_drule(:,:,i) = decisionRule(image_raw(:, : ,i), goodMu(i), goodSigma(i), badMu(i), badSigma(i));
    end
    
    if(preview)
        showPreview(image_raw, image_pure, image_drule);
    end
end

%% A function to apply the decision rule on an image
%  The output is a binairy image that is 1 when the chance is larger that a
%  pixel is a cucumber that that it is not.
function image = decisionRule(values, goodMu, goodSigma, badMu, badSigma)
    image = zeros(length(values:1),length(values));
    for x = 1:length(values)
        for y = 1:length(values(:,1))
             bad  = normpdf(values(y,x), badMu, badSigma);
             good = normpdf(values(y,x), goodMu, goodSigma);
             image(y,x) = good < bad;
        end
    end
end

function showPreview(image_raw, image_pure, image_drule)
    subplot(4, 3, 1);
    imagesc(image_raw);
    image_r = image_raw(:, : ,1);
    image_g = image_raw(:, : ,2);
	image_b = image_raw(:, : ,3);
    
    subplot(4, 3, 2);
    
    image_size = size(image_r);
    z = zeros(image_size);

    %% Show RGB components
    subplot(4, 3, 4);
    imagesc(cat(3,image_r,z,z));
    subplot(4, 3, 5);
    imagesc(cat(3,z,image_g,z));
    subplot(4, 3, 6);
    imagesc(cat(3,z,z,image_b));

    %% Show Pure components
    subplot(4, 3, 7);
    imagesc(cat(3,image_pure(:, : ,1),z,z));
    subplot(4, 3, 8);
    imagesc(cat(3,z,image_pure(:, : ,2),z));
    subplot(4, 3, 9);
    imagesc(cat(3,z,z,image_pure(:, : ,3)));

    %% Show images using decision rule
    subplot(4, 3, 10);
    imagesc(cat(3,image_drule(:, : ,1),z,z));
    subplot(4, 3, 11);
    imagesc(cat(3,z,image_drule(:, : ,2),z));
    subplot(4, 3, 12);
    imagesc(cat(3,z,z,image_drule(:, : ,3)));
    
    subplot(4, 3, 3);
    imagesc(image_drule);
end