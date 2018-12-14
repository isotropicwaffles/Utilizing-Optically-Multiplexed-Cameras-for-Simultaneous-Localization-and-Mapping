% Demultiplexing from depth test
laplacian_flag = true; %add smoothnesse constraints
smoothness_weight = 5; %weight to multiply laplacians by

blur_flag = false; %Add blurring effect to the disparity constraint
blur_all = false; %Blur original image as well
blur_sigma =2; %Degree of blur
interp_method =2; %1 Nearest neighbor pixel assocations, 2 linear interp pixel assocations
direct_method_flag = false;%Direct inversion (true) or LSQR (false)
%% Input images

% load source image

left_img1 = cv.imread(['test_images' filesep 'Butterfly_World_Camera_L.png'], ...
    'Grayscale',true);
right_img1 = cv.imread(['test_images' filesep 'Butterfly_World_Camera_R.png'], ...
    'Grayscale',true);

imgScene = left_img1/2 + right_img1/2;

depth_img = cv.imread(['test_images' filesep 'Butterfly_World_Camera_L_Depth_1cm.png'], ...
    'Grayscale',true);

%% Estimate disparity
fx = (1024 * 0.5) / tan(1.3333*60 * 0.5 * pi/180);
baseline = .1;
depth_img = double(depth_img)*.01;
disparity = fx*baseline./depth_img';

%Visualize Depth and image
figure; pcolor(disparity');shading flat; title('Depth');
figure; pcolor(flipud(imgScene));shading flat; title('Multiplexed Image');colormap('gray');


%Determine how the indices change when we vectorize the matrix
[indx, indy] = meshgrid(1:size(imgScene,2),1:size(imgScene,1));
indx = indx';
indy = indy';
vect_indx = indx(:);
vect_indy = indy(:);

%determine pixel offsets (1 is right image, 2 is left image)
deltax1 = vect_indx-(disparity(:));
deltax2 = vect_indx+(disparity(:));

%figure;pcolor(reshape(deltax1,size(trans_imgScene,1),size(trans_imgScene,2)));shading flat

switch interp_method
    
    case 1 %Nearest Neighbor
        %reference indices
        ref_indx1 = (1:numel(imgScene))';
        ref_indx2 = (1:numel(imgScene))';
        
        %Set values to be assigned to offset positions
        element_value1 = ones(size(deltax1));
        element_value2 = ones(size(deltax2));
        rounded_deltax1 = round(deltax1);
        rounded_deltax2 = round(deltax2);
        element_value1(rounded_deltax1 <=0) = 0;
        element_value2(rounded_deltax2 >size(imgScene,2)) = 0;
        %Correct for the reference indx
        ref_indx1(rounded_deltax1 <=0) = nan;%dummy index.
        ref_indx2((rounded_deltax2 >size(imgScene,2))) = nan;%dummy index.
        
        rounded_deltax1 = ref_indx1-round(disparity(:));
        rounded_deltax1(isnan(rounded_deltax1)) =numel(imgScene);
        rounded_deltax2 = ref_indx2+round(disparity(:));
        rounded_deltax2(isnan(rounded_deltax2)) =1;
        
        
        %generate sparse matrices
        delta_constraint_1 = spconvert([(1:numel(imgScene))' rounded_deltax1 element_value1]);
        delta_constraint_2 = spconvert([(1:numel(imgScene))' rounded_deltax2 element_value2]);
        
    case 2
        %reference indices
        ref_indx1_ceil = (1:numel(imgScene))';
        ref_indx1_floor = (1:numel(imgScene))';
        ref_indx2_ceil = (1:numel(imgScene))';
        ref_indx2_floor = (1:numel(imgScene))';
        
        %determin ceiling and floors
        ceil_deltax1 = ceil(deltax1);
        ceil_deltax2 = ceil(deltax2);
        floor_deltax1 = floor(deltax1);
        floor_deltax2 = floor(deltax2);
        
        %Determine weights to apply
        element_ceil_value1  = abs(ceil_deltax1-deltax1).*ones(size(deltax1));
        element_ceil_value2  = abs(ceil_deltax2-deltax2).*ones(size(deltax2));
        element_floor_value1 = abs(floor_deltax1-deltax1).*ones(size(deltax1));
        element_floor_value2 = abs(floor_deltax2-deltax2).*ones(size(deltax2));
        
        %handle values off picture/boundaries
        element_ceil_value1(floor_deltax1 <=0 & ceil_deltax1 >0) = 1;
        element_ceil_value2(floor_deltax2 <=0 & ceil_deltax2 >0) = 1;
        element_ceil_value1(ceil_deltax1 <=0 ) = 0;
        element_ceil_value2(ceil_deltax2 <=0 ) = 0;
        
        element_floor_value1(floor_deltax1 <=0) = 0;
        element_floor_value2(floor_deltax2 <=0) = 0;
        
        element_floor_value1(ceil_deltax1 >size(imgScene,2) & floor_deltax1 <=size(imgScene,2)) = 1;
        element_floor_value2(ceil_deltax2 >size(imgScene,2)& floor_deltax2 <=size(imgScene,2)) = 1;
        element_ceil_value1(ceil_deltax1 >size(imgScene,2)) = 0;
        element_ceil_value2(ceil_deltax2 >size(imgScene,2)) = 0;
        element_floor_value1(floor_deltax1 >size(imgScene,2)) = 0;
        element_floor_value2(floor_deltax2 >size(imgScene,2)) = 0;
        
        
        %Handle determining indices
        ref_indx1_ceil(ceil_deltax1 <=0) = nan;
        ref_indx2_ceil(ceil_deltax2 <=0) = nan;
        ref_indx1_ceil(ceil_deltax1 >size(imgScene,2)) =nan;
        ref_indx2_ceil(ceil_deltax2 >size(imgScene,2)) =nan;
        
        ref_indx1_floor(floor_deltax1 <=0) = nan;
        ref_indx2_floor(floor_deltax2 <=0) = nan;
        ref_indx1_floor((floor_deltax1 >size(imgScene,2))) = nan;
        ref_indx2_floor((floor_deltax2 >size(imgScene,2))) = nan;
        
        
        ceil_deltax1 = ceil(ref_indx1_ceil-(disparity(:)));
        ceil_deltax2 = ceil(ref_indx2_ceil+(disparity(:)));
        floor_deltax1 = floor(ref_indx1_floor-(disparity(:)));
        floor_deltax2 = floor(ref_indx2_floor+(disparity(:)));
        
        ceil_deltax1(isnan(ceil_deltax1)) = numel(imgScene);%dummy index
        ceil_deltax2(isnan(ceil_deltax2)) =1;%dummy index
        floor_deltax1(isnan(floor_deltax1)) = numel(imgScene);%dummy index
        floor_deltax2(isnan(floor_deltax2)) =1;%dummy index
        
        %generate sparse matrices
        delta_constraint_1 = spconvert([(1:numel(imgScene))' ceil_deltax1 element_ceil_value1]) + spconvert([(1:numel(imgScene))' floor_deltax1 element_floor_value1]);
        delta_constraint_2 = spconvert([(1:numel(imgScene))' ceil_deltax2 element_ceil_value2]) + spconvert([(1:numel(imgScene))' floor_deltax2 element_floor_value2]);
        
end

%I want the vectorize array to have the horizontal indices sequentially so
%I need to transpose the image before vectorization
trans_imgScene = imgScene';

%Image reconstruction with blurr
if blur_flag
    %Create a blurred image if requested
    imgScene_blurred = imgaussfilt(trans_imgScene,blur_sigma);
else
    %Don't blur
    imgScene_blurred = trans_imgScene;%
end

%% Definition of multiplexing constraint (from the multiplexing paper)
A1 = [speye(numel(imgScene)) speye(numel(imgScene))];
%Check if you want this constraint to be blurred
if blur_all
    %Blur the multiplexing constraint
    b1 = double(imgScene_blurred(:));
else
    %Don't blur the multiplexing constraint
    b1 = double(trans_imgScene(:));
end

%% Disparity constraint
A2 = [delta_constraint_2 speye(numel(imgScene))];
A3 = [speye(numel(imgScene)) delta_constraint_1];

%Disparity constraint
b2 = double(imgScene_blurred(:));
b3 = double(imgScene_blurred(:));


%% Impose smoothness constraint for reconstruction 

%Smoothness constraint
b4 = zeros(size(b3));
b5 = zeros(size(b3));
b6 = zeros(size(b3));
b7 = zeros(size(b3));

%% calculate laplacian constraint in x direction
temp_val_right =  spconvert([(1:numel(imgScene))' [(1:numel(imgScene)-1)'+1; numel(imgScene)]  [-ones(numel(imgScene)-1,1); 0]]);
temp_val_left =  spconvert([[(1:numel(imgScene))'; numel(imgScene)] [1; (2:numel(imgScene))'-1; numel(imgScene)] [0; -ones(numel(imgScene)-1,1);0]]);

laplacian_x = 2*speye(numel(imgScene)) + temp_val_right+temp_val_left;
%Filter out the boundary cases
laplacian_x(vect_indx == indx(1,1) | vect_indx == indx(end,end),: ) = [];
b4(vect_indx == indx(1,1) | vect_indx == indx(end,end)) = [];
b5(vect_indx == indx(1,1) | vect_indx == indx(end,end)) = [];
%build A matrix
A4 = [laplacian_x 0*speye(size(laplacian_x))];
A5 = [0*speye(size(laplacian_x)) laplacian_x];

%% calculate laplacian constraint in y direction

up_indices = (1:numel(imgScene)- size(imgScene,2));
down_indices = (size(imgScene,2):numel(imgScene));

temp_val_up =  spconvert([(1:numel(imgScene))' [numel(imgScene)*ones(size(imgScene,2),1);(1:numel(imgScene)-size(imgScene,2))']  [0*(1:size(imgScene,2))'; -ones(numel(imgScene)-size(imgScene,2),1)]]);
temp_val_down =  spconvert([(1:numel(imgScene))' [(size(imgScene,2)+1:numel(imgScene))'; numel(imgScene)*ones(size(imgScene,2),1)]  [-ones(numel(imgScene)-size(imgScene,2),1);0*(1:size(imgScene,2))']]);

laplacian_y = 2*speye(numel(imgScene)) + temp_val_up+temp_val_down;

%Filter out the boundary cases
laplacian_y(vect_indy == indy(1,1) | vect_indy == indy(end,end),: ) = [];
b6(vect_indy == indy(1,1) | vect_indy == indy(end,end)) = [];
b7(vect_indy == indy(1,1) | vect_indy == indy(end,end)) = [];
%build A matrix
A6 = [laplacian_y 0*speye(size(laplacian_y))];
A7 = [0*speye(size(laplacian_y)) laplacian_y];




switch interp_method
    
    case 1 %Nearsest Neighbor
        
        A2(rounded_deltax1 == numel(imgScene),:) = [];
        A3(rounded_deltax2 == 1,:) = [];
        b2(rounded_deltax1 == numel(imgScene)) = [];
        b3(rounded_deltax2 == 1) = [];
    
    case 2 %linear combination
        
        A2(floor_deltax1 == numel(imgScene) & ceil_deltax1 == numel(imgScene),:) = [];
        A3(floor_deltax2 ==1 & ceil_deltax2 == 1,:) = [];
        b2(floor_deltax1 == numel(imgScene) & ceil_deltax1 == numel(imgScene)) = [];
        b3(floor_deltax2 ==1 & ceil_deltax2 == 1) = [];
        
end
%
%Check if smoothness should be included as constraint
if laplacian_flag
   %Create the final A matrix for inversion. Smoothness weight will effect
   %how much weight the smoothness constraint should have
    A_prime = [A1;A2;A3;smoothness_weight*A4;smoothness_weight*A5;smoothness_weight*A6;smoothness_weight*A7];
   
    b = [b1; b2; b3;b4;b5;b6;b7];
else
    %Create the final A matrix for inversion without smoothness constraint
    A_prime = [A1;A2;A3];
    
    b = [b1; b2; b3];%Use the same image thrice
end

%Perform the inversion
if direct_method_flag
    test_img = [A_prime]\[b];
else
    tol = 1e-8;
    maxit = 100;
    test_img = lsqr(A_prime,b,tol,maxit);
end

%Form images from results
test_img1 = reshape(test_img(1:end/2),size(trans_imgScene,1),size(trans_imgScene,2));
test_img2 = reshape(test_img(end/2+1:end),size(trans_imgScene,1),size(trans_imgScene,2));
%plot results
figure;pcolor(flipud(test_img1'));shading flat; colormap('gray'); title('Reconstructed FOV 1');
figure;pcolor(flipud(test_img2'));shading flat; colormap('gray'); title('Reconstructed FOV 2');
figure;pcolor(flipud((double(trans_imgScene)-test_img1)'));shading flat; colormap('gray'); title('FOV 1 Delta from Multiplexed Image');
figure;pcolor(flipud((double(trans_imgScene)-test_img2)'));shading flat; colormap('gray'); title('FOV 2 Delta from Multiplexed Image');
