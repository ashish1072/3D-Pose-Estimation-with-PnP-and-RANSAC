function [points] = project3d2image(points, cam_intrinsics, cam_in_world_orientations, cam_in_world_locations)
%   points: 3xn
%   cam_intrinsics: from cameraParameters() function
%   cam_in_world_orientations - orientaions of cameras in world coordinate system
%   cam_in_world_locations - locationss of cameras in world coordinate system

reprojected_imageSample_XYZ = cam_in_world_orientations*points - cam_in_world_orientations*cam_in_world_locations.';
reprojected_imageSample_uvw = cam_intrinsics.IntrinsicMatrix.'*reprojected_imageSample_XYZ;
reprojected_imageSample_pixel = reprojected_imageSample_uvw ./ reprojected_imageSample_uvw(3,:);
points = reprojected_imageSample_pixel(1:2, :);
end

