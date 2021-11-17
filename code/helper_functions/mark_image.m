function I = mark_image(path, max_points)
% Annotating points on an image
% Arguments: 
% path - the path to a directory with images
% max_points - the number of points of 3D object to mark
% Output:
% 3-dimensional array
% 1st dimension - (1:max_points) index of a point
% 2nd dimension - (1:2) x, y coordinates
% 3rd dimension - (1:maximum number of files) index of a file in directory
% Usage:
% path = '../data/init_texture';
% I = mark_image(path, 8);

path_template = fullfile(path, '*.JPG');

img_files = dir(path_template);
num_files = length(img_files);

I = NaN(max_points, 2, length(img_files));

for i=1:num_files
    
    fprintf('Image: %d \n', i);
    img_data = imread(fullfile(path, img_files(i).name));
    fig = figure;
    image(img_data);
    hold on;
    
    for j=1:max_points
        
        [maxValY, maxValX, ~]  = size(img_data);
        
        for k=1:j
            if ~isnan(I(k, 1, i))
                plot(I(k, 1, i), I(k, 2, i),'x', 'LineWidth', 1, 'MarkerSize', 5, 'Color', 'b')
                text(I(k, 1, i) + maxValX * 0.01, I(k, 2, i) - maxValY * 0.01, char(num2str(k)), 'FontSize',12, 'Color', 'b')
            end  
        end
        
        title_message = ['Image %d  Point %d \n' ...
            'Mark point %d and press Enter/Return to submit the point. \n' ...
            'If point is not in the picture, press Enter/Return without selecting the point. \n' ...
            'Use Delete to deselect a point.'];
        title(sprintf(title_message, i, j, j));
        
        [x,y] = getpts();
        
        if isempty(x) || isempty(y)
%             close(fig);
            continue
        end
        
        if ((x <= maxValX) && (x >= 0)) || ((y <= maxValY) && (y >= 0))
            I(j, 1, i) = x(1);
            I(j, 2, i) = y(1);  
            fprintf('Point: %d X: %f Y: %f \n', j, x(1), y(1));
        end
        
        
    end
    close(fig);
end

