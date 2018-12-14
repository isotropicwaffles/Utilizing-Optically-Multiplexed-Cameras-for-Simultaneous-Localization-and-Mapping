close all;
clear ;
baseline_filename = 'ampersand_maxSpeed1p0_poses_centered.csv';
stereo_filename = ['./mono_multi_colorfilter/KeyFrameTrajectory.txt'];
formatSpec = '%f %f %f';
sizePoints = [8 Inf];

fileID = fopen(stereo_filename, 'r');
stereo_points = fscanf(fileID, formatSpec, sizePoints);
stereo_points = stereo_points';
fclose(fileID);

baseline_points = csvread(baseline_filename);


baseline_points(1:1806,:) = [];
baseline_points(:,3) = baseline_points(:,3) - baseline_points(1,3);
baseline_points(:,4) = baseline_points(:,4) - baseline_points(1,4);
baseline_points(:,2) = baseline_points(:,2)  - baseline_points(1,2);
bx = baseline_points(:,3);
by = baseline_points(:,4);
bz = baseline_points(:,2);

b = [baseline_points(:,1) bx by bz];

% %Stereo OM

% fx = 4.8;
% ox = 0;
% fy = 4.6;
% oy =0;
 %fz = 4.6;
% oz = 0;

% % Mono OM
% fx = 4.2;
% ox = 0;
% fy = 6;
% oy =0;
% fz = 6;
% oz = 0;
% Mono SF OM
% fx = 4.2;
% ox = 0;
% fy = 3.6;
% oy =0;
% fz = 4.9;

% %STereo BL
% fx = 1.3;
% ox = 0;
% fy = 1.1;
% oy =0;
% fz = 1.1;
% oz = 0;


% %Stereo Filter OM
% fx = 1.3;
% ox = 0;
% fy = 1.1;
% oy =0;
% fz = 1.1;
% oz = 0;

%None
 fx = 4.2;
% ox = 0;
 fy = 3.6;
% oy =0;
 fz = 4.9;
% oz = 0;



b(:,1) = ((b(:,1)-b(1,1))/1e6)*4;
s = [stereo_points(:,1) ...
           stereo_points(:,2) .* fx                ...
           stereo_points(:,3) .* fy   ...
           stereo_points(:,4).* fz];
       
     
s(:,2) = s(:,2) - s(1,2) + interp1(b(:,1),b(:,2),s(1,1)); 
s(:,3) = s(:,3) - s(1,3) + interp1(b(:,1),b(:,3),s(1,1)); 
s(:,4) = s(:,4) - s(1,4) + interp1(b(:,1),b(:,4),s(1,1)); 
RMSE_stereo = mRMSE(b, s)

