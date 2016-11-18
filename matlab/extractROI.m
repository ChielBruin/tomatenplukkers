% This function converts the ROI table to the correct file structure
% (see: https://sgsai.blogspot.nl/2016/02/training-faster-r-cnn-on-custom-dataset.html)
% -table > The exported table
% -path > The location to store the annotation files
% -preview > Show the result when true
function extractROI(table, path, preview)
    images = table.imageFilename;
    m = size(table.Properties.VariableNames);
    classes = m(2)-1;
    
    % For each image
    for i = 1:size(images)
        annotations = []; 
        % For each class
        for c = 1:classes
            A = table{i, c+1};
            
            if iscell(A)    % Multiple boxes
                a = reshape(cell2mat(A),[],4);
                s = size(a);
                t = zeros(s(1), 1);
                for j = 1:s(1)
                    t(j) = c;
                end
                annotations = [annotations; [t a]];
            else            % Single box
                a = [c A];
                annotations = [annotations; a];
            end
        end
        
        % Write the results to a file
        t = images(i);
        image = t{1,1}(1:end-3);
        
        file = [path '/' [image(find(image=='/',1,'last')+1:end) 'txt']];
        [f,r] = fopen(file, 'wt' );
        if (isempty(r))
            % Write
            l = size(annotations);
            for p=1:l(1)
                row = annotations(p, 1:5);
                fprintf(f, '%d %d %d %d %d\n', row);
            end
            fclose(f);
   
            if preview
                fprintf('\nfile: %s', file)
                type (file)
            end
        else
           % ERROR!
           fprintf('\nfile "%s could not be opened: %s"', file, r)
        end
    end