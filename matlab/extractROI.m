% This function converts the ROI table to the correct file structure
% (see: https://sgsai.blogspot.nl/2016/02/training-faster-r-cnn-on-custom-dataset.html)
% -table > The exported table
% -path > The location to store the annotation files
% -preview > Show the result when true
function extractROI(table, path, preview)
    images = table.imageFilename;
    tmp = size(table.Properties.VariableNames);
    classes = tmp(2)-1;
    
    % For each image
    fixes = [0, 0, 0, 0];
    for i = 1:size(images)
        annotations = []; 
        % For each class
        for c = 1:classes
            data = table{i, c+1};
            
            if iscell(data)    % Multiple boxes
                tmp = reshape(cell2mat(data),[],4);
                tmpSize = size(tmp);
                classCol = zeros(tmpSize(1), 1);
                for j = 1:tmpSize(1)
                    classCol(j) = c;
                end
                annotations = [annotations; [classCol tmp]];
            else            % Single box
                annotations = [annotations; [c data]];
            end
        end
        
        % Write the results to a file
        tmp = images(i);
        image = tmp{1,1}(1:end-3);
        
        file = [path '/' [image(find(image=='/',1,'last')+1:end) 'txt']];
        img_size = size(imread(strcat(image,'jpg')));
        
        [f,r] = fopen(file, 'wt' );
        if (isempty(r))
            % Write
            l = size(annotations);
            for p=1:l(1)
                row = annotations(p, 1:5);
                if (row(2) < 0) % x
                    row(2) = 1;
                    disp('Fixed negative x');
                    fixes(1) = fixes(1) + 1;
                end
                
                if (row(3) < 0) % y
                    row(3) = 1;
                    disp('Fixed negative y');
                    fixes(2) = fixes(2) + 1;
                end
                
                if (row(4) + row(2) > img_size(2)) % width
                    row(4) = img_size(2) - row(2);                    
                    disp('Fixed too wide');
                    fixes(3) = fixes(3) + 1;
                end
                
                if (row(5) + row(3) > img_size(1)) % height
                    row(5) = img_size(1) - row(3);
                    disp('Fixed too high');
                    fixes(4) = fixes(4) + 1;
                end
                
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
    fixes