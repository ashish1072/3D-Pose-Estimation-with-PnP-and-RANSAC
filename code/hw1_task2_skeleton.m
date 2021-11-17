clear
clc
close all
addpath('helper_functions')
run('vlfeat-0.9.21/toolbox/vl_setup')

%% Setup
% path to the images folder
path_img_dir = '../data/detection';
% path to object ply file
object_path = '../data/teabox.ply';

% Read the object's geometry 
% Here vertices correspond to object's corners and faces are triangles
[vertices, faces] = read_ply(object_path);

% Load the SIFT model from the previous task
load('sift_model.mat');


% TODO: setup camera intrinsic parameters using cameraParameters()
IntrinsicMatrix = [2960.37845 0 0; 0 2960.37845 0; 1841.68855 1235.23369 1];
camera_params = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

%% Get all filenames in images folder

FolderInfo = dir(fullfile(path_img_dir, '*.JPG'));
Filenames = fullfile(path_img_dir, {FolderInfo.name} );
num_files = length(Filenames);
% num_files = 1;

%% Match SIFT features of new images to the SIFT model with features computed in the task 1
% You should use VLFeat function vl_ubcmatch()

% Place SIFT keypoints and descriptors of new images here
keypoints=cell(num_files,1);
descriptors=cell(num_files,1);
% Place matches between new SIFT features and SIFT features from the SIFT
% model here
sift_matches=cell(num_files,1);

% Default threshold for SIFT keypoints matching: 1.5 
% When taking higher value, match is only recognized if similarity is very high
threshold_ubcmatch = 1.5; 
% 
% for i=1:num_files
%     fprintf('Calculating and matching sift features for image: %d \n', i)
%     
% %   TODO: Prepare the image (img) for vl_sift() function
%     img = single(rgb2gray(imread(char(Filenames(i)))));
% 
%     [keypoints{i}, descriptors{i}] = vl_sift(img);
% %   Match features between SIFT model and SIFT features from new image
%     sift_matches{i} = vl_ubcmatch(descriptors{i}, model.descriptors, threshold_ubcmatch); 
% end
% 
% 
% % Save sift features, descriptors and matches and load them when you rerun the code to save time
% save('sift_matches.mat', 'sift_matches');
% save('detection_keypoints.mat', 'keypoints')
% save('detection_descriptors.mat', 'descriptors')


load('sift_matches.mat')
load('detection_keypoints.mat')
load('detection_descriptors.mat')


%% PnP and RANSAC 
% Implement the RANSAC algorithm featuring also the following arguments:
% Reprojection error threshold for inlier selection - 'threshold_ransac'  
% Number of RANSAC iterations - 'ransac_iterations'

% Pseudocode
% i Randomly select a sample of 4 data points from S and estimate the pose using PnP.
% ii Determine the set of data points Si from all 2D-3D correspondences 
%   where the reprojection error (Euclidean distance) is below the threshold (threshold_ransac). 
%   The set Si is the consensus set of the sample and defines the inliers of S.
% iii If the number of inliers is greater than we have seen so far,
%   re-estimate the pose using Si and store it with the corresponding number of inliers.
% iv Repeat the above mentioned procedure for N iterations (ransac_iterations).

% For PnP you can use estimateWorldCameraPose() function
% but only use it with 4 points and set the 'MaxReprojectionError' to the
% value of 10000 so that all these 4 points are considered to be inliers

% Place camera orientations, locations and best inliers set for every image here
cam_in_world_orientations = zeros(3,3,num_files);
cam_in_world_locations = zeros(1,3,num_files);
best_inliers_set = cell(num_files, 1);

ransac_iterations = 1000; 
threshold_ransac = 4;
no_points = 4;
max_rep_error = 1000;

for i = 1:num_files
    fprintf('Running PnP+RANSAC for image: %d \n', i)
    % Initializing best inliers indexes
%     best_inliers_set{i} = 1:size(sift_matches{i},2);
    best_count = 4;
     
    % Random selection of 4 points data points 
    for j=1:ransac_iterations
        sel =  randperm(size(sift_matches{i},2),no_points);
        sel_sift_matches = sift_matches{i}(:,sel);
        image_points = keypoints{i}(1:2, sel_sift_matches(1,:));
        model_sift_points = model.coord3d';
        world_points = model_sift_points(1:3,sel_sift_matches(2,:));
        try
        [rot,tran] = estimateWorldCameraPose(image_points', world_points', camera_params, 'MaxReprojectionError', max_rep_error);  
        catch
    %       fprintf('error in iteration: %d \n',j);
            continue;
        end
    % Projection of 3D points and Euclidean distance computation
        [points] = project3d2image(model_sift_points(:, sift_matches{i}(2,:)), camera_params, rot, tran);
        coor_diff = points - keypoints{i}(1:2, sift_matches{i}(1,:));
        euclid_dist = vecnorm(coor_diff);

    % Finding and storing indexes and number of inliers with euclidean distances less than ransac threshold
        inlier_index = find(euclid_dist <= threshold_ransac);
        num_inliers = length(inlier_index);
        if (num_inliers>best_count)
           best_count = num_inliers;
           best_inliers_set{i} = inlier_index;
           sel_sift_matches = sift_matches{i}(:,inlier_index);
           image_points = keypoints{i}(1:2, sel_sift_matches(1,:));
           model_sift_points = model.coord3d';
           world_points = model_sift_points(1:3,sel_sift_matches(2,:));  
    % Re-estimation of pose using inliers
           [cam_in_world_orientations(:,:,i),cam_in_world_locations(:, :, i)] = estimateWorldCameraPose(image_points', world_points', camera_params, 'MaxReprojectionError', max_rep_error);  
%            cam_in_world_orientations(:,:,i) = r;
%            cam_in_world_locations(:, :, i) = t;
           fprintf('maximum number of inliers: %d \n', best_count);   
        end
    end    
end


%% Visualize inliers and the bounding box

% You can use the visualizations below or create your own one
% But be sure to present the bounding boxes drawn on the image to verify
% the camera pose

edges = [[1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7]
    [2, 4, 5, 3, 6, 4, 7, 8, 6, 8, 7, 8]];

for i=1:num_files
    
    figure()
    imshow(char(Filenames(i)), 'InitialMagnification', 'fit');
    title(sprintf('Image: %d', i))
    hold on
    
%   Plot inliers set
%     PlotInlierOutlier(best_inliers_set{i}, camera_params, sift_matches{i}, model.coord3d, keypoints{i}, cam_in_world_orientations(:,:,i), cam_in_world_locations(:,:,i))
%   Plot bounding box
    points = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:, :, i));
    for j=1:12
    plot(points(1, edges(:, j)), points(2, edges(:,j)), 'color', 'b');
    end
    hold off;
end