function fig = visualise_cameras(vertices, edges, cam_in_world_orientations, cam_in_world_locations)
% vertices - vertices of the object Nx3 double
% edges - edges of the object 2xN
% cam_in_world_orientations - orientaions of cameras in world coordinate
% system
% cam_in_world_locations - locationss of cameras in world coordinate
% system


% Visualize computed camera poses
fig = figure();

num_files = length(cam_in_world_orientations);
pcshow(vertices,'VerticalAxis','Y','VerticalAxisDir','down', 'MarkerSize',30) % plot the corners of the box

% connect vertices
for i=1:length(edges)
    hold on
    plot3([vertices(edges(1,i),1), vertices(edges(2, i),1)],[vertices(edges(1, i),2), vertices(edges(2,i),2)],[vertices(edges(1,i),3), vertices(edges(2, i),3)], 'color', 'blue')
end

for i=1:num_files
    hold on
    plotCamera('Size',0.01,'Orientation',cam_in_world_orientations(:,:,i),'Location',...
        cam_in_world_locations(:,:,i)) % plot the cameras
end

xlabel('x');
ylabel('y');
zlabel('z');
hold off

end