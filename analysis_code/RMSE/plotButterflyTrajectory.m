close all;
clear;
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



% %Stereo OM

 fx_stereoom = 4.8;
% ox = 0;
 fy_stereoom = 3.5;
% oy =0;
fz_stereoom = 4.6;
% oz = 0;

% % Mono OM
 fxmono = 4.2;
% ox = 0;
 fymono = 5;
% oy =0;
 fzmono = 6;
% oz = 0;

% Mono SF OM
 fxmono_sf = 4.2;
% ox = 0;
 fymono_sf = 3.5;
% oy =0;
 fzmono_sf = 4.9;

% %STereo BL
 fx = 1.3;
% ox = 0;
 fy = 1.1;
% oy =0;
 fz = 1.1;
% oz = 0;


% %Stereo Filter OM
 fxmstereo_sf = 1.3;
% ox = 0;
 fystereo_sf = 1.1;
% oy =0;
 fzstereo_sf = 1.1;
% oz = 0;

baseline_points(1:1806,:) = [];
baseline_points(:,1) = ((baseline_points(:,1)-baseline_points(1,1))/1e6)*4;

baseline_points(:,3) = baseline_points(:,3) - baseline_points(1,3);
baseline_points(:,4) = baseline_points(:,4) - baseline_points(1,4);
baseline_points(:,2) = baseline_points(:,2) - baseline_points(1,2);

sx = sx -sx(1) + interp1(baseline_points(:,1),baseline_points(:,3),stereo_points(1,1)); 
sy = sy -sy(1) + interp1(baseline_points(:,1),baseline_points(:,4),stereo_points(1,1));
sz = sz -sz(1) + interp1(baseline_points(:,1),baseline_points(:,2),stereo_points(1,1)) ;

mx = mx -mx(1) + interp1(baseline_points(:,1),baseline_points(:,3),multi_points(1,1)); 
my = my -my(1) + interp1(baseline_points(:,1),baseline_points(:,4),multi_points(1,1));
mz = mz -mz(1) + interp1(baseline_points(:,1),baseline_points(:,2),multi_points(1,1)) ;


multi_color_points(:,2) = multi_color_points(:,2) -multi_color_points(1,2) + interp1(baseline_points(:,1),baseline_points(:,3),multi_color_points(1,1)); 
multi_color_points(:,3) = multi_color_points(:,3) -multi_color_points(1,3) + interp1(baseline_points(:,1),baseline_points(:,4),multi_color_points(1,1)); 
multi_color_points(:,4) = multi_color_points(:,4) -multi_color_points(1,4) + interp1(baseline_points(:,1),baseline_points(:,2),multi_color_points(1,1)); 


figure; hold on; grid on;
scatter3(baseline_points(:,3), baseline_points(:,4) , baseline_points(:,2), 1, 'b');
scatter3(sx.*fx, sy.*fy, sz.*fz, 1, 'm');
scatter3(mx .* fx_stereoom , my .* fy_stereoom , mz .* fz_stereoom , 1, 'r');
scatter3(multi_color_points(:,2)*fxmstereo_sf, multi_color_points(:,3)*fystereo_sf, multi_color_points(:,4)*fzstereo_sf, 1, 'c');
title("Estimated Stereo SLAM Trajectories");
legend("Ground truth", "Stereo", "Stereo OM", "Stereo OM SF");
xlabel('Estimated X Pos');
ylabel('Estimated Y Pos');
zlabel('Estimated Z Pos');
 view(3);


 mono_points(:,2) = mono_points(:,2) -mono_points(1,2) + interp1(baseline_points(:,1),baseline_points(:,3),mono_points(1,1)); 
mono_points(:,3) = mono_points(:,3) -mono_points(1,3) + interp1(baseline_points(:,1),baseline_points(:,4),mono_points(1,1)); 
mono_points(:,4) = mono_points(:,4) -mono_points(1,4) + interp1(baseline_points(:,1),baseline_points(:,2),mono_points(1,1)); 

mono_color_points(:,2) = mono_color_points(:,2) -mono_color_points(1,2) + interp1(baseline_points(:,1),baseline_points(:,3),mono_color_points(1,1)); 
mono_color_points(:,3) = mono_color_points(:,3) -mono_color_points(1,3) + interp1(baseline_points(:,1),baseline_points(:,4),mono_color_points(1,1)); 
mono_color_points(:,4) = mono_color_points(:,4) -mono_color_points(1,4) + interp1(baseline_points(:,1),baseline_points(:,2),mono_color_points(1,1)); 


 
figure; hold on; grid on;
mono_scale = 3.8;
scatter3(baseline_points(:,3) , baseline_points(:,4), baseline_points(:,2) , 1, 'b');

plot3(mono_points(:,2) .* fxmono, mono_points(:,3) .* fymono , mono_points(:,4) .* fzmono, '-x', 'Color', 'g');
mono_scale = 3.4;
plot3(mono_color_points(:,2) .*fxmono_sf, mono_color_points(:,3) .* fymono_sf+1 , mono_color_points(:,4) .* fzmono_sf , '-x', 'Color', 'k');
title("Estimated Monocular SLAM Trajectories");
legend("Ground truth", "Mono", "Mono SF");
xlabel('Estimated X Pos');
ylabel('Estimated Y Pos');
zlabel('Estimated Z Pos');



 view(3);