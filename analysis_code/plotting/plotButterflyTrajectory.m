close all;

scenario = 'butterfly';

baseline_filename = '../external/AgileDrones/OfflineRenderClient/Butterfly_World_data/ampersand_maxSpeed1p0_poses_centered.csv';
stereo_filename = [scenario '_stereo/CameraTrajectory.txt'];
multiplexed_filename = [scenario '_stereo_multiplexed/CameraTrajectory.txt'];
mono_filename = [scenario '_mono/KeyFrameTrajectory.txt'];
multi_color_filename = [scenario '_stereo_multi_color/CameraTrajectory.txt'];
mono_color_filename = [scenario '_mono_color/KeyFrameTrajectory.txt'];

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
scatter3(baseline_points(:,3) - 0.249323, baseline_points(:,4) - 0.912902, baseline_points(:,2) + 3.710597, 1, 'b');
scatter3(sx, sy - 0.05, sz, 1, 'm');
scatter3(mx .* fx + ox, my .* fy + oy, mz .* fz + oz, 1, 'r');
scatter3(multi_color_points(:,2), multi_color_points(:,3), multi_color_points(:,4), 1, 'c');
title("Estimated Stereo SLAM Trajectories");
legend("Ground truth", "Stereo", "Stereo OM", "Stereo OM SF");
xlabel('Estimated X Pos');
ylabel('Estimated Y Pos');
zlabel('Estimated Z Pos');
% view(3);


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