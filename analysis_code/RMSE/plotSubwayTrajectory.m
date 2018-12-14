close all;

scenario = 'subway';

stereo_filename = [scenario '_stereo/CameraTrajectory.txt'];
multiplexed_filename = [scenario '_stereo_multiplexed/CameraTrajectory.txt'];

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

sx = stereo_points(:,2);
sy = stereo_points(:,3);
sz = stereo_points(:,4);

mx = multi_points(:,2);
my = multi_points(:,3);
mz = multi_points(:,4);

fx = 3.15;
ox = 0.1;
fy = -3.7;
oy = -0.15;
fz = 4;
oz = -1.5;

figure; hold on; grid on;
scatter3(sx, sy, sz, 1, 'b');
scatter3(mx .* fx + ox, my .* fy + oy, mz .* fz + oz, 1, 'r');
xlabel('Estimated X Coordinate (m)');
ylabel('Estimated Y Coordinate (m)');
zlabel('Estimated Z Coordinate (m)');