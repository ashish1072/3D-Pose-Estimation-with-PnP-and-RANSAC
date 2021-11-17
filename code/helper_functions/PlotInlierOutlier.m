function [  ] = PlotInlierOutlier(inlier_idx, cam_intrinsics, matches, model_coords, sift_keypoints, cam_in_world_orientations, cam_in_world_locations )

% inlier_idx: refers to column in matches matrix
% cam_intrinsics: from cameraParameters() function
% matches: matches from vl_ubcmatch()
% model_coords: coordinates from the SIFT model
% sift_keypoints: sift keypoints from vl_sift() for new images
% cam_in_world_orientations - orientaions of cameras in world coordinate system
% cam_in_world_locations - locationss of cameras in world coordinate system


    %plot detected feature
    inlier_feature_image_ind = matches(1,inlier_idx);
    feature_points = sift_keypoints(1:2, inlier_feature_image_ind);
    plot(feature_points(1,:), feature_points(2,:), 'ob');
    %plot projection
    inlier_model_ind = matches(2,inlier_idx);
    coords3d = model_coords(inlier_model_ind,:);
    image_coords = project3d2image(coords3d', cam_intrinsics, cam_in_world_orientations, cam_in_world_locations);
    hold on;
    plot(image_coords(1,:), image_coords(2,:), 'xr');
    % plot error vectors
    errors = feature_points - image_coords;
    for j = 1:length(image_coords)
        hold on;
        quiver(image_coords(1,j), image_coords(2,j),errors(1,j), errors(2,j),0, 'MaxHeadSize', 0.02, 'Color', 'b');
    end
end

