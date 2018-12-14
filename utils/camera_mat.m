%Define Camera relationship
% intrinsic camera parameters
res_x = 1024;%Pixel Resolution in x direction
res_y = 768;%Pixel Resolution in y direction
fov_y = 60;%FOV in y direction
fov_x = fov_y*res_x/res_y;%FOV in x direction
fx = (res_x * 0.5) / tan(fov_x * 0.5 * pi/180);
fy = (res_y * 0.5) / tan(fov_y * 0.5 * pi/180);
yaw = 90;
pitch =0;
roll = 0;

%Camera A intrinsic params
Ka = [ fx   0  res_x/2;
    0  fy  res_y/2;
    0   0        1];

%Camera B intrinsic params
Kb = [ fx   0  res_x/2;
    0  fy  res_y/2;
    0   0        1];