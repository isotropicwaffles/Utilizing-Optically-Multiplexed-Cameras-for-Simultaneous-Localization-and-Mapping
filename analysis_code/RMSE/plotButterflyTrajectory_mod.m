close all;

scenario = 'butterfly';

baseline_filename = './ampersand_maxSpeed1p0_poses_centered.csv';
stereo_filename = ['./stereo_baseline/CameraTrajectory.txt'];
multiplexed_filename = ['./multiplexed_stereo/CameraTrajectory.txt'];
mono_filename = ['./mono_baseline/KeyFrameTrajectory.txt'];
multi_color_filename = ['./stereo_mult_colorfilter/CameraTrajectory.txt'];
mono_color_filename = ['./mono_multi_colorfilter/KeyFrameTrajectory.txt'];

formatSpec = '%f %f %f';
sizePoints = [8 Inf];

fileID = fopen(stereo_filename, 'r');
stereo_points = fscanf(fileID, formatSpec, sizePoints);
stereo_points = stereo_points';
fclose(fileID);

fileID = fopen(multiplexed_filename, 'r');
multi_points = fscanf(fileID, formatSpec, sizePoints);
multi_points = multi_points';
fclose(fileID);

fileID = fopen(mono_filename, 'r');
mono_points = fscanf(fileID, formatSpec, sizePoints);
mono_points = mono_points';
fclose(fileID);

fileID = fopen(multi_color_filename, 'r');
multi_color_points = fscanf(fileID, formatSpec, sizePoints);
multi_color_points = multi_color_points';
fclose(fileID);

fileID = fopen(mono_color_filename, 'r');
mono_color_points = fscanf(fileID, formatSpec, sizePoints);
mono_color_points = mono_color_points';
fclose(fileID);

baseline_points = csvread(baseline_filename);

sx = stereo_points(:,2);
sy = stereo_points(:,3);
sz = stereo_points(:,4);

mx = multi_points(:,2);
my = multi_points(:,3);
mz = multi_points(:,4);

fx = 4.75;
ox = 0;
fy = 4.6;
oy = 0.05;
fz = 4.6;
oz = 0;

figure; hold on; grid on;
title("Estimated Stereo SLAM Trajectories");
%legend("Ground truth", "Stereo", "Stereo OM", "Stereo OM SF");
xlabel('Estimated X Pos');
ylabel('Estimated Y Pos');
zlabel('Estimated Z Pos');
baseline_points(1:1806,:) = [];
baseline_points(:,3) = baseline_points(:,3) - baseline_points(1,3);
baseline_points(:,4) = baseline_points(:,4) - baseline_points(1,4);
baseline_points(:,2) = baseline_points(:,2)  - baseline_points(1,2);

sx = sx -sx(1) + baseline_points(1,3); 
sy = sy -sy(1) + baseline_points(1,4); 
sz = sz -sz(1) + baseline_points(1,2); 

for ii = 1:size(baseline_points,1)
    curr_time =(baseline_points(ii,1)-baseline_points(1,1))/1e6;
    title(['Estimated Stereo SLAM Trajectories Time: ' num2str(ii)]);

temp1=scatter3(baseline_points(ii,3), baseline_points(ii,4), baseline_points(ii,2), 1, 'b');
temp2=scatter3(sx(ii), sy(ii), sz(ii), 1, 'm');
pause(0.1);

axis([-1 1 -1 1 -1 1]);
view(3)
%%delete(temp1); delete(temp2);
end
scatter3(mx .* fx + ox, my .* fy + oy, mz .* fz + oz, 1, 'r');
scatter3(multi_color_points(:,2), multi_color_points(:,3), multi_color_points(:,4), 1, 'c');

% ;


figure; hold on; grid on;
mono_scale = 3.8;
plot3(mono_points(:,2) .* mono_scale, mono_points(:,3) .* mono_scale - 0.4, mono_points(:,4) .* mono_scale * 1.4, '-x', 'Color', 'g');
mono_scale = 3.4;
plot3(mono_color_points(:,2) .* mono_scale, mono_color_points(:,3) .* mono_scale - 0.15, mono_color_points(:,4) .* mono_scale .* 1.5, '-x', 'Color', 'k');
scatter3(baseline_points(:,3) - 0.249323, baseline_points(:,4) - 0.912902, baseline_points(:,2) + 3.710597, 1, 'b');
title("Estimated Monocular SLAM Trajectories");
legend("Ground truth", "Mono", "Mono SF");
xlabel('Estimated X Pos');
ylabel('Estimated Y Pos');
zlabel('Estimated Z Pos');
% view(3);