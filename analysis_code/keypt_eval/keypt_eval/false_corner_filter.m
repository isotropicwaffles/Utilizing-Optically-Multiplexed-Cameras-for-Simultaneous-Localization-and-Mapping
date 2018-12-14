%% 6.869 Project
% Creates keypoint mask over potential false corners in grayscale image.
% The keypoint mask can be provided as an argument to an OpenCV feature
% detector function, which will prevent the feature detector from finding
% keypoints at locations where false corners potentially exist.
% False corners are those not of objects in real world but those 
% produced from overlapping of objects with different luminances from
% different fields of view in the multiplexed image space. The ORB feature 
% detector correctly associates the "same" false corners between two 
% frames, but since those false corners don't stay fixed relative to either
% object, using them for estimating camera pose will corrupt the estimation.

% MATLAB Coder options:
% -config:lib
% -args {coder.typeof(0,[inf inf])}

function kp_mask = false_corner_filter(grayscale_img) %#codegen
%% Detect edges in image

% Img gradients
dmdx = conv2(grayscale_img, [-1 0 1; -2 0 2; -1 0 1], 'same');
dmdy = conv2(grayscale_img, [-1 0 1; -2 0 2; -1 0 1]', 'same');

% Calculate edge strength and set to zero at img boundaries
edge_mag = sqrt(dmdx.^2+dmdy.^2);
edge_mag(1:end,1) = 0;
edge_mag(1:end,end) = 0;
edge_mag(1,1:end) = 0;
edge_mag(end,1:end) = 0;

% Produce binary img of edges and skeletonize it
edges = edge_mag > 30;
% edges = edge_mag > 2*mean(edge_mag(:));
edge_skel = bwmorph(edges,'skel');
edge_skel = bwmorph(edge_skel,'thin');
edge_skel = bwmorph(edge_skel,'spur');

%% Identify potential false corners and generate keypoint mask over them

% Find initial set of branchpoints (both T/Y and X junctions) in edge skeleton
edge_branchpoints = bwmorph(edge_skel,'branchpoints');
edge_branchpoints = bwmorph(edge_branchpoints,'shrink');

% Convolve edge skeleton with 7x7 ring kernel
edge_skel_conv = conv2(double(edge_skel), ...
    ones(7)-padarray(ones(5),[1 1]),'same');

% Zero out convolution at all pixel locations except at branchpoints
edge_skel_conv(~edge_branchpoints) = 0;

% Only keep branchpoints whose convolution results are greater than 3
% This occurs when more than three branches cross the 7x7 ring kernel, 
% which indicates the presence of an X junction
edge_cross_loc = edge_skel_conv > 3;

% Generate keypoint mask
kp_mask = uint8(conv2(double(edge_cross_loc),ones(7),'same') == 0);

% Plot skeletonized edges and keypoint masks
% h(1) = subplot(1,2,1);
% imshow(edge_skel)
% axis equal tight
% h(2) = subplot(1,2,2);
% imshow(edge_cross_loc)
% axis equal tight
% linkaxes(h)
