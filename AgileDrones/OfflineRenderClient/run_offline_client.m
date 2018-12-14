%Simple Script run offline render client
%Only one camera can be ran at a time, so multiple cameras will require
%multiple runs
%./OfflineRenderClient Butterfly_World pose_file param_file

%All parameters are defined in a json file
%You can use this function to estimate the camera quaternions to input to
%the json file
yaw = -120;
pitch =0;
roll = 0;
camera_quaternion = angle2quat((pi/180)*roll, (pi/180)*pitch, (pi/180)*yaw,'XYZ');
angle2dcm((pi/180)*roll, (pi/180)*pitch, (pi/180)*yaw,'XYZ')

%% 'Drone' Pose information
%pose_file = './NYC_Subway_data/mouse_maxSpeed1p0_poses_centered.csv';
pose_file = './Butterfly_World_data/ampersand_maxSpeed1p0_poses_centered.csv';

%Ultra Wide FOV run
%param_file = './NYC_Subway_data/ultra_wide_camera.json';
%param_file = './NYC_Subway_data/ultra_wide_camera.json';
%param_file = './NYC_Subway_data/ultra_wide_camera_reverse.json';
%param_file = './Butterfly_World_data/stereo_left_camera_baseline_10cm_depth4.json';
param_file = './Butterfly_World_data/stereo_left_camera_baseline_10cm.json';

%Stero Runs
%param_file = './NYC_Subway_data/stereo_right_camera_baseline_10cm.json';
%param_file = './NYC_Subway_data/stereo_left_camera_baseline_10cm.json';


%% Build run command 
command = ['OfflineRenderClient ' param_file ' ' pose_file];
system(command,'-echo')
