% Calculate the essential matrix.

clear all
close all

%% Options
OPTS_FEATURE = 'ORB';   % detector: ORB, BRISK, AKAZE, KAZE, SIFT, SURF
OPTS_FLANN = false;       % matcher: FLANN or Brute Force
OPTS_KNN_MATCH = false;  % matcher method: match or knnMatch (k=2)
flip_img_b = false;
%% Define Camera relationship
% intrinsic camera parameters
res_x = 768;
res_y = 768;
fov_y = 60;
fov_x = fov_y*res_x/res_y;
fx = (res_x * 0.5) / tan(fov_x * 0.5 * pi/180);
fy = (res_y * 0.5) / tan(fov_y * 0.5 * pi/180);
yaw = -180;
pitch =0;
roll = 0;



% load source image
A_img1 = cv.imread(['../test_images' filesep 'Butterfly_World_Camera_Forward.png'], ...
    'Grayscale',true);
B_img1 = cv.imread(['../test_images' filesep 'Butterfly_World_Camera_Reverse.png'], ...
    'Grayscale',true);

A_img2 = cv.imread(['../test_images' filesep 'Butterfly_World_Camera_Forward2.png'], ...
    'Grayscale',true);
B_img2 = cv.imread(['../test_images' filesep 'Butterfly_World_Camera_Reverse2.png'], ...
    'Grayscale',true);

%Camera A intrinsic params
Ka = [ fx   0  res_x/2-1;
    0  fy  res_y/2-1;
    0   0        1];

%Camera B intrinsic params
Kb = [ fx   0  res_x/2-1;
    0  fy  res_y/2-1;
    0   0        1];
%This is the camera matrix of a flipped image
if flip_img_b
    
    Kb = [-1 1 1;
        1 1 1;
        1 1 1].*Kb;
    
end


%Rotation matrix from camera A -> camera B
%Rab = angle2dcm((pi/180)*roll, (pi/180)*pitch, (pi/180)*yaw,'XYZ');

ax = roll* (pi/180);
ay = yaw * (pi/180);
az = roll* (pi/180);

Rx = [ 1    0            0;
    0    cos(ax)     -sin(ax);
    0    sin(ax)      cos(ax) ];
Ry = [ cos(ay)    0     sin(ay);
    0          1     0;
    -sin(ay)    0     cos(ay) ];
Rz = [ cos(az)    -sin(az)   0;
    sin(az)     cos(az)   0;
    0           0         1 ];
Rab = Rx * Ry * Rz;


%% Input images
if flip_img_b
    B_img1 = fliplr(B_img1);
end
multi_img1 = A_img1/2 + B_img1/2;

if flip_img_b
    B_img2 = fliplr(B_img2);
end
multi_img2 = A_img2/2 + B_img2/2;

%Run through the non-multiplexed and multiplexed cases for comparison
test_cases = {'Multiplexed','As','Bs'};

%These store the correspondences of the multiplexed, A images, and B images
ptsObj_to_plot = cell(1,3);
ptsScene_to_plot = cell(1,3);

for jj = 1:3
    
    test_case = test_cases{jj};
    
    switch test_case
        case 'Multiplexed'
            %Test Multiplex Images
            imgObj = multi_img1;
            imgScene = multi_img2;
        case 'As'
            %Test Demultiplexed Left Images
            imgObj = A_img1;
            imgScene = A_img2;
        case 'Bs'
            %Test Demultiplexed Right Images
            imgObj = B_img1;
            imgScene = B_img2;
    end
    
    %% Step 1: Detect the keypoints and extract descriptors using SURF
    switch upper(OPTS_FEATURE)
        case 'SURF'
            detector = cv.SURF('HessianThreshold',400);
        case 'SIFT'
            detector = cv.SIFT();
        case 'ORB'
            detector = cv.ORB();
            %Some settings that seem to work okay
            detector.MaxFeatures = 1200;
            %             detector.PatchSize = 25;
            %             detector.EdgeThreshold = 19;
            detector.FastThreshold = 15;
            detector.PatchSize = 31;
            detector.EdgeThreshold = 29;
        case 'BRISK'
            detector = cv.BRISK();
        case 'AKAZE'
            detector = cv.AKAZE();
        case 'KAZE'
            detector = cv.KAZE();
        otherwise
            error('unrecognized feature: %s', OPTS_FEATURE)
    end
    display(detector)
    
    %%Detect keypoints
    [keyObj,featObj] = detector.detectAndCompute(imgObj);
    [keyScene,featScene] = detector.detectAndCompute(imgScene);
    Objpts = [keyObj.pt];
    Objpts = reshape(Objpts,2,length(Objpts)/2)';
    
    %Store keypoints
    Scenepts = [keyScene.pt];
    Scenepts = reshape(Scenepts,2,length(Scenepts)/2)';
    
    fprintf('object: %d keypoints\n', numel(keyObj));
    fprintf('scene: %d keypoints\n', numel(keyScene));
    whos featObj featScene
    
    %% Step 2: Matching descriptor vectors using FLANN matcher
    if OPTS_FLANN
        if ~isempty(strfind(detector.defaultNorm(), 'Hamming'))
            opts = {'LSH', 'TableNumber',6, 'KeySize',12, 'MultiProbeLevel',1};
        else
            opts = {'KDTree', 'Trees',5};
        end
        matcher = cv.DescriptorMatcher('FlannBasedMatcher', 'Index',opts);
    else
        matcher = cv.DescriptorMatcher('BFMatcher', ...
            'NormType',detector.defaultNorm());
    end
    display(matcher)
    
    
    if OPTS_KNN_MATCH
        matches = matcher.knnMatch(featObj, featScene, 2);
    else
        matches = matcher.match(featObj, featScene);
    end
    fprintf('%d good matches\n', numel(matches));
    
    
    %% Filter matches and keep only "good" ones
    if OPTS_KNN_MATCH
        % ratio test
        if iscell(matches)
            
            zero_idx = cellfun(@(m)(numel(m) == 0),matches);
            matches(zero_idx) = [];
        end
        dists = cellfun(@(m) m(1).distance, matches);
        
        
        idx = cellfun(@(m) (numel(m) == 2) && ...
            (m(1).distance < 0.75 * m(2).distance), matches);
        matches = cellfun(@(m) m(2), matches(idx));
        
        
    else
        if iscell(matches)
            
            zero_idx = cellfun(@(m)(numel(m) == 0),matches);
            matches(zero_idx) = [];
        end
        % distance less than k*min_dist
        dists = [matches.distance];
        cutoff = 3 * min(dists);
        matches = matches(dists <= cutoff);
        fprintf('Min dist = %f\nMax dist = %f\nCutoff = %f\n', ...
            min(dists), max(dists), cutoff);
    end
    fprintf('%d good matches\n', numel(matches));
    
    
    %%
    % Get the keypoints from the good matches
    % (Note: indices in C are zero-based while MATLAB are one-based)
    ptsObj = cat(1, keyObj([matches.queryIdx]+1).pt)';
    ptsScene = cat(1, keyScene([matches.trainIdx]+1).pt)';
    whos ptsObj ptsScene
    
    
   %% If it's a multiplexed image then try to compute the essential matrices through RanSAC
    if (jj == 1)
        [best_E_mats, best_K_mats,best_res,best_min_res,best_min_res_indx] =...
            computeEssentialMatRanSAC_mod(ptsObj,ptsScene,Ka,Kb,Rab,8000);
        
        best_min_res_indx (best_min_res>3)=[];
        ptsObj(:,best_min_res>3) =[];
        ptsScene(:,best_min_res>3) = [];
    end
    
    %Store the correspondences
    ptsObj_to_plot{jj} = ptsObj;
    ptsScene_to_plot{jj} = ptsScene;
end

%% Display points on the images for visualization
figure; subplot(1,2,1), imshow(multi_img1, []); hold on;

%Loop through the sets of correspondences to plot them
for jj = 3:-1:1
    ptsObj = ptsObj_to_plot{jj};
    
    for i=1:length(ptsObj)
        x = round(ptsObj(1,i));     y = round(ptsObj(2,i));
        
        switch jj
            case 1
                %This selects the color of which multiplexed FOV set is
                %expected to be associated with
                switch best_min_res_indx(i)
                    case 1
                        color = 'r'; %Ka -> Ka
                    case 2
                        color = 'b'; %Kb -> Kb
                    case 3
                        color = 'm';%Kb -> Ka
                    case 4
                        color = 'c';%Ka -> Kb
                end
                rectangle('Position', [x-4 y-4 8 8], 'EdgeColor', color);
                
            case 2
                %Plot the non-muliplexed image A points
                scatter(x,y,'xr');
            case 3
                %Plot the non-multiplexed image B points
                scatter(x,y,'xb');
        end
    end
    
end

%Plot the second multiplexed image
subplot(1,2,2), imshow(multi_img2, []); hold on;


%Loop through the sets of correspondences to plot them of image 2
for jj = 3:-1:1
    ptsScene = ptsScene_to_plot{jj};
    
    for i=1:length(ptsScene)
        x = round(ptsScene(1,i));     y = round(ptsScene(2,i));
        
        switch jj
            case 1
                switch best_min_res_indx(i)
                    case 1
                        color = 'r';
                    case 2
                        color = 'b';
                    case 3
                        color = 'm';
                    case 4
                        color = 'c';
                end
                rectangle('Position', [x-4 y-4 8 8], 'EdgeColor', color);
                
            case 2
                scatter(x,y,'xr');
            case 3
                scatter(x,y,'xb');
        end
    end
    
end

%% Draw epipolar lines on image 1
for i=1:length(ptsScene)
    subplot(1,2,1)
    % The product l=E*p2 is the equation of the epipolar line corresponding
    % to p2, in the first image.  Here, l=(a,b,c), and the equation of the
    % line is ax + by + c = 0.
    %Get the corresponding E for the correspondence
    E = best_E_mats{best_min_res_indx(i)};

    %Calculate the epipolar line from image2 -> image1
    l = cv.computeCorrespondEpilines(ptsScene(1:2,i)', inv(best_K_mats{2,best_min_res_indx(i)})'*E*inv(best_K_mats{1,best_min_res_indx(i)}),'WhichImage',2);
    
    %Determine distance of image1 point from epipolar line 
    distance = point_to_line(ptsObj(:,i),l);
    
    fprintf('Distance is %f to point %d\n', distance, i);
    
    % Let's find two points on this line. First set x=-1 and solve
    % for y, then set x=1 and solve for y.
    pLine0 = [0,-l(3)/l(2)];
    pLine1 = [res_x ,((-l(3)-l(1).*res_x)/l(2))];

%This selects the color of which multiplexed FOV set is
                %expected to be associated with
                switch best_min_res_indx(i)
                    case 1
                        color = 'r'; %Ka -> Ka
                    case 2
                        color = 'b'; %Kb -> Kb
                    case 3
                        color = 'm';%Kb -> Ka
                    case 4
                        color = 'c';%Ka -> Kb
                end
    
    %Plot the epipolar line on image 1
    temp_line = line([pLine0(1) pLine1(1)], [pLine0(2) pLine1(2)], 'Color', color);
    
    %Plot the point on image 1
    temp_point1 = scatter(ptsObj(1,i),ptsObj(2,i),50,'d','MarkerEdgeColor','k','MarkerFaceColor',color);
    
    subplot(1,2,2)
    %Plot the point on image 2
    temp_point2 = scatter(ptsScene(1,i),ptsScene(2,i),50,'d','MarkerEdgeColor','k','MarkerFaceColor',color);
    
    pause;
    
    delete(temp_line); delete(temp_point1);delete(temp_point2);
end

%% Draw epipolar lines on image 2
for i=1:length(ptsObj)
    subplot(1,2,2)
    % The product l=E'*p1 is the equation of the epipolar line corresponding
    % to p1, in the second image.  Here, l=(a,b,c), and the equation of the
    % line is ax + by + c = 0.
    E = best_E_mats{best_min_res_indx(i)};
    
    l = cv.computeCorrespondEpilines(ptsObj(1:2,i)', inv(best_K_mats{2,best_min_res_indx(i)})'*E*inv(best_K_mats{1,best_min_res_indx(i)}),'WhichImage',1);
    
    distance = point_to_line(ptsScene(:,i),l);
    
    
    fprintf('Distance is %f to point %d\n', distance, i);
    
    % Let's find two points on this line. First set x=-1 and solve
    % for y, then set x=1 and solve for y.
    pLine0 = [0,-l(3)/l(2)];
    pLine1 = [res_x ,((-l(3)-l(1).*res_x)/l(2))];
    
%This selects the color of which multiplexed FOV set is
                %expected to be associated with
                switch best_min_res_indx(i)
                    case 1
                        color = 'r'; %Ka -> Ka
                    case 2
                        color = 'b'; %Kb -> Kb
                    case 3
                        color = 'm';%Kb -> Ka
                    case 4
                        color = 'c';%Ka -> Kb
                end
    %Plot the epipolar line on image 1    
    temp_line = line([pLine0(1) pLine1(1)], [pLine0(2) pLine1(2)], 'Color', color);
    %Plot the point on image 1    
    temp_point1 = scatter(ptsScene(1,i),ptsScene(2,i),50,'d','MarkerEdgeColor','k','MarkerFaceColor',color);
    
    subplot(1,2,1)
    %Plot the epipolar line on image 1    
    temp_point2 = scatter(ptsObj(1,i),ptsObj(2,i),50,'d','MarkerEdgeColor','k','MarkerFaceColor',color);
    
    
    pause;
    delete(temp_line); delete(temp_point1); delete(temp_point2);
    
end

%This function calculates the distance a point is from a line from a line
function d = point_to_line(pt, line)

d = abs(line(1).*pt(1) + line(2).*pt(2) + line(3))./...
    sqrt(line(1)*line(1)+line(2)*line(2));
end
