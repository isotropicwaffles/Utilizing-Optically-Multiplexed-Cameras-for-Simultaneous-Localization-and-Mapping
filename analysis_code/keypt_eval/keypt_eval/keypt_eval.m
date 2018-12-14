%% 6.869 Project

clear
close all

% Specify path to mexopencv directory
addpath(genpath('C:\Users\tomch\6.869_git\mexopencv'))

%% Parameters

% Feature detector: ORB, BRISK, AKAZE, KAZE, SIFT, SURF
OPTS_FEATURE = {'ORB'};

%% Load FOV images and create optically multiplexed (OM) image

% Load grayscale FOV 1
img1 = double(cv.imread(['../../data/butterfly_mono_color/cam_forward/', ...
    '1524892379467282_Butterfly_World_Camera_UW_Forward.png'],'Grayscale',true));

% Load grayscale FOV 2
img2 = double(cv.imread(['../../data/butterfly_mono_color/cam_right/', ...
    '1524892379458949_Butterfly_World_Camera_UW_Right.png'],'Grayscale',true));

% Create optically multiplexed (OM) img
imgm = img1/2 + img2/2;

% Intensity-scale imgs for plotting purposes
img1sc = uint8(255*img1/max(img1(:)));
img2sc = uint8(255*img2/max(img2(:)));
imgmsc = uint8(255*imgm/max(imgm(:)));

%% Create keypoint mask over potential false corners in OM img

kp_mask = false_corner_filter(imgm);

for i=1:numel(OPTS_FEATURE)
%% Find keypoints

    % Create feature detector object
    switch upper(OPTS_FEATURE{i})
        case 'SURF'
            detector = cv.SURF('HessianThreshold',400);
        case 'SIFT'
            detector = cv.SIFT();
        case 'ORB'
            detector = cv.ORB();
            % Some settings that seem to work okay
            detector.MaxFeatures = 1e6;
            % detector.PatchSize = 25;
            % detector.EdgeThreshold = 19;
    %         detector.FastThreshold = 15;
    %         detector.PatchSize = 31;
    %         detector.EdgeThreshold = 29;
        case 'BRISK'
            detector = cv.BRISK();
        case 'AKAZE'
            detector = cv.AKAZE();
        case 'KAZE'
            detector = cv.KAZE();
        otherwise
            error('unrecognized feature detector: %s', OPTS_FEATURE{i})
    end
    display(detector)

    % FOV 1 keypoints
    kp1 = detector.detect(img1);

    % FOV 2 keypoints
    kp2 = detector.detect(img2);

    % OM img keypoints with and without mask
    kpm_nomask = detector.detect(imgm);
    kpm = detector.detect(imgm,'Mask',kp_mask);

    %% Draw keypoints

    % % FOV 1
    % kp1img = cv.drawKeypoints(img1sc,kp1,'Color',255*[1 0 0]);	% red
    % figure(1)
    % imshow(kp1img)
    % title('FOV 1')
    % 
    % % FOV 2
    % kp2img = cv.drawKeypoints(img2sc,kp2,'Color',255*[1 0 0]);	% red
    % figure(2)
    % imshow(kp2img)
    % title('FOV 2')
    % 
    % % OM img without mask
    % kpm_nomaskimg = cv.drawKeypoints(imgmsc,kpm_nomask,'Color',255*[1 0 0]);    % red
    % figure(3)
    % imshow(kpm_nomaskimg)
    % title('OM Img without Masking')
    % 
    % % OM img with mask
    % kpm_img = cv.drawKeypoints(imgmsc,kpm,'Color',255*[1 0 0]);	% red
    % figure(4)
    % imshow(kpm_img)
    % title('OM Img with Masking')

    %% Calculate keypoint statistics
    % Below code rounds all keypoint locations to nearest pixel

    img_size = size(img1);

    % Total keypoints in OM img with and without mask
    kpm_nomask_idx = [kpm_nomask.pt];
    kpm_nomask_idx = sub2ind(img_size,round(kpm_nomask_idx(1:2:end)), ...
        round(kpm_nomask_idx(2:2:end)));
    kpm_nomask_total = numel(unique(kpm_nomask_idx));
    kpm_idx = [kpm.pt];
    kpm_idx = sub2ind(img_size,round(kpm_idx(1:2:end)), ...
        round(kpm_idx(2:2:end)));
    kpm_total = numel(unique(kpm_idx));

    % Keypoints shared between FOV 1 and OM img with and without mask
    % Make matrix with ones at kp1 locations
    % Count number of identically-located keypoints in kpm and kpm_nomask
    kp1_idx = [kp1.pt];
    kp1_idx = sub2ind(img_size,round(kp1_idx(1:2:end)), ...
        round(kp1_idx(2:2:end)));
    % kp1_total = numel(unique(kp1_idx));
    kp1_mat = zeros(img_size);
    kp1_mat(kp1_idx) = 1;
    % Expand point locations
    % kp1_mat = conv2(double(kp1_mat),ones(3),'same') > 0;
    kp1_kpm_nomask_shared = sum(kp1_mat(kpm_nomask_idx));
    kp1_kpm_shared = sum(kp1_mat(kpm_idx));

    % Keypoints shared between FOV 2 and OM img with and without mask
    kp2_idx = [kp2.pt];
    kp2_idx = sub2ind(img_size,round(kp2_idx(1:2:end)), ...
        round(kp2_idx(2:2:end)));
    % kp2_total = numel(unique(kp2_idx));
    kp2_mat = zeros(img_size);
    kp2_mat(kp2_idx) = 1;
    % Expand point locations
    % kp2_mat = conv2(double(kp2_mat),ones(3),'same') > 0;
    kp2_kpm_nomask_shared = sum(kp2_mat(kpm_nomask_idx));
    kp2_kpm_shared = sum(kp2_mat(kpm_idx));

    %% Metrics

    % Number and percentage of keypoints in OM img without mask
    % that are shared with FOV 1 and 2
    kpm_nomask_shared_total(i) = kp1_kpm_nomask_shared+kp2_kpm_nomask_shared;
    kpm_nomask_shared_percent(i) = 100*kpm_nomask_shared_total/kpm_nomask_total;
    fprintf('OM img without false corner masking:\n')
    fprintf('%u keypoints shared with FOV 1 and 2\n', ...
        kpm_nomask_shared_total(i))
    fprintf('%.1f%% keypoints shared with FOV 1 and 2\n', ...
        kpm_nomask_shared_percent(i))

    % Number and percentage of keypoints in OM ing with mask
    % that are shared with FOV 1 and 2
    kpm_shared_total(i) = kp1_kpm_shared+kp2_kpm_shared;
    kpm_shared_percent(i) = 100*kpm_shared_total/kpm_total;
    fprintf('OM img with false corner masking:\n')
    fprintf('%u keypoints shared with FOV 1 and 2\n', ...
        kpm_shared_total(i))
    fprintf('%.1f%% keypoints shared with FOV 1 and 2\n', ...
        kpm_shared_percent(i))
end

%% Generate bar graphs

x = categorical({'Without FC Masking', ...
    'With FC Masking'});

figure
h = bar(x,[kpm_nomask_shared_percent; kpm_shared_percent],.9);
legend(h,OPTS_FEATURE)
ylabel('Percent (%)')
title('% Correct Features in OM Image')
grid on

figure
h = bar(x,[kpm_nomask_shared_total; kpm_shared_total],.9);
legend(h,OPTS_FEATURE)
ylabel('Number of Features')
title('Number of Correct Features in OM Image')
grid on
