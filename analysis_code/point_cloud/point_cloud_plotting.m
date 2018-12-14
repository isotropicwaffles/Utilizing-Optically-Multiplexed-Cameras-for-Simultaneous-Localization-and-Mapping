filename = 'PointMap.txt';
filename2 = 'PointMap2.txt';
filename3 = 'PointMap_mono.txt';

data = importdata(filename);
data2 = importdata(filename2);
data3 = importdata(filename3);


traj1 = 'KeyFrameTrajectory.txt';
traj2 = 'KeyFrameTrajectory2.txt';
traj_data1 = importdata(traj1);
traj_data2 = importdata(traj2);

ref_point1 = traj_data1(abs(traj_data1(:,1) - 317.5667)<0.01,2:4);
ref_point2 = traj_data2(abs(traj_data2(:,1) - 317.5667)<0.01,2:4);

data_2_norm = sum((traj_data2(:,2:4)-ref_point2).^2,2);
data_1_norm = sum((traj_data1(:,2:4)-ref_point1).^2,2);

data_1_norm_aligned = interp1(traj_data1(:,1),data_1_norm,traj_data2(:,1));


mean_ratio=nanmean(data_2_norm);
median_ratio = nanmedian(data_2_norm);
data2 = 3*data2/mean_ratio;

figure; title('3d point cloud (Scale Arbitrary for Monocular)');
scatter3(data(:,1),data(:,2),data(:,3),'ro'); axis equal;
xlabel('X');ylabel('Y');zlabel('Z');hold on;

title('3d point cloud (Scale Arbitrary for Monocular)');
scatter3(data2(:,1),data2(:,2),data2(:,3),'bo'); axis equal;
xlabel('X');ylabel('Y');zlabel('Z');



pc = pointCloud(data);
pc3 = pointCloud(data3);

pcwrite(pc,'object3d.ply','Encoding','ascii');

figure;
cm = colormap('winter');
pcshow(pc);hold on; xlabel('X'); ylabel('Y');zlabel('Z');
yaw = -200;
pitch =80;
roll = 0;
camera_quaternion = angle2quat((pi/180)*roll, (pi/180)*pitch, (pi/180)*yaw,'XYZ');
Rx2 = angle2dcm((pi/180)*roll, (pi/180)*pitch, (pi/180)*yaw,'XYZ');
pc2 = pointCloud((Rx2*data2')');

pc3 = pointCloud([data; (Rx2*data2')' + [1 0 0.5]]);

pc4 =pointCloud(data3);
figure;
colormap('hot')
pcshow(pc3);xlabel('X'); ylabel('Y');zlabel('Z');
xlim([-1.5 2]); title('2xFOV Multiplexed with Spectral Filters');
xlabel('X');ylabel('Y');zlabel('Z');

figure;
colormap('hot')
pcshow(pc4);xlabel('X'); ylabel('Y');zlabel('Z');
axis([-2 2 -2 2 -2 2]); title('Singel FOV Monocular')
xlabel('X');ylabel('Y');zlabel('Z');