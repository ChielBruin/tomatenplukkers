function findaaar(imagePath, table)
    faceDetector = vision.CascadeObjectDetector(table);

    % Read a video frame and run the face detector.
    img_raw  = imread(imagePath);
    bbox     = step(faceDetector, img_raw);

    % Draw the returned bounding box around the detected face.
    img_raw = insertShape(img_raw, 'Rectangle', bbox);
    figure; 
     imshow(img_raw); 
     title('Detected face');
end