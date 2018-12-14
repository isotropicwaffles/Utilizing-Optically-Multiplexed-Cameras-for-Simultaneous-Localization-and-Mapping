function [best_E_mats, best_K_mats,best_res,best_min_res,best_min_res_indx] = computeEssentialMatRanSAC(p1,p2,Ka,Kb,Rab,numiter)
%Use ransac to calculate essential matrix
%Input
%   p1 - image 1 points [2xn]
%   p2 - image 2 points [2xn]
%   Ka - intrisnic matrix of camera A [3x3]
%   Kb - intrisnic matrix of camera B [3x3]
%   Rab - rotation matrix that goes from camera A to camera B [3x3]
%   numiter - number of iterations [1]
%Ouput
%   E - Essential Matrix Estimation [3x3]


if nargin == 3
    numiter =10;
end

num_pts = length(p1);

poss_indices = 1:num_pts;
best_res = inf;

p1_3d = [p1; ones(1,length(p1))];
p2_3d = [p2; ones(1,length(p2))];

for ii = 1:numiter
    %Let's assume we randomly select 8 points and they are good points within
    %one FOV
    
    sample_idx=randsample(poss_indices,8);
    
    
    %figure out how to handle fundamental matrix!!!!!
    %Let's assume it came frome the same FOV
    
    %Check all FOV for inliers
    %Let's test all of the possible solutions and find the ones that are the
    %best
    
    res_array = nan(size(p2,2),4);
    E_mats = cell(1,4);
    K_mats = cell(2,4);
    
    for kk = 1:4
        
        samp_p1 = p1(:,sample_idx);
        samp_p2 = p2(:,sample_idx);
        
        %Four possible solutions
        switch kk
            case 1
                samp_p1 =Ka\[samp_p1;ones(1,size(samp_p1,2))];
                samp_p2 =Ka\[samp_p2;ones(1,size(samp_p2,2))];
            case 2
                samp_p1 =Kb\[samp_p1;ones(1,size(samp_p1,2))];
                samp_p2 =Kb\[samp_p2;ones(1,size(samp_p2,2))];
            case 3
                samp_p1 =Kb\[samp_p1;ones(1,size(samp_p1,2))];
                samp_p2 =Ka\[samp_p2;ones(1,size(samp_p2,2))];
            case 4
                samp_p1 =Kb\[samp_p1;ones(1,size(samp_p1,2))];
                samp_p2 =Ka\[samp_p2;ones(1,size(samp_p2,2))];
        end
        
        [E] = cv.findEssentialMat(samp_p1(1:2,:)',samp_p2(1:2,:)');

        for jj = 1:4
            
            %Four possible solutions
            switch kk
                case 1
                    switch jj
                        case 1
                            
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Ka;
                            
                        case 2
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Kb;
                        case 3
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E*Rab;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Kb;
                        case 4
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E/Rab;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Ka;
                    end
                    
                case 2
                    
                    switch jj
                        case 1
                            
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Ka;
                            
                        case 2
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Kb;
                        case 3
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E*Rab;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Kb;
                        case 4
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E/Rab;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Ka;
                    end
                    
                case 3
                    
                    switch jj
                        case 1
                            
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E*Rab;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Ka;
                            
                        case 2
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E*Rab;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Kb;
                        case 3
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E*Rab*Rab;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Kb;
                        case 4
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Ka;
                    end
                    
                case 4
                    
                    switch jj
                        case 1
                            
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E/Rab;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Ka;
                            
                        case 2
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E/Rab;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Kb;
                        case 3
                            temp_p1 = Ka\p1_3d;
                            temp_p2 = Kb\p2_3d;
                            temp_E = E;
                            K_mats{1,jj} = Ka;
                            K_mats{2,jj} = Kb;
                        case 4
                            temp_p1 = Kb\p1_3d;
                            temp_p2 = Ka\p2_3d;
                            temp_E = E/Rab/Rab;
                            K_mats{1,jj} = Kb;
                            K_mats{2,jj} = Ka;
                    end
            end
            
            E_mats{jj} = temp_E;
            
            %Normalize so that it doesn't choose essential matrices
            %that are near zero
            
            %temp_E = temp_E./sqrt(sum(sum(temp_E.^2)));
            for ii=1:length(p2)
                % The product l=E*p2 is the equation of the epipolar line corresponding
                % to p2, in the first image.  Here, l=(a,b,c), and the equation of the
                % line is ax + by + c = 0.
                %l = E * p2(:,i);
                
                % Calculate residual error.  The product p1'*E*p2 should = 0.  The
                % difference is the residual.
                res_array(ii,jj)= abs(temp_p2(:,ii)' * temp_E *temp_p1(:,ii));
                

%                 
                Ex1 = temp_E * temp_p2(:,ii);
                Etx2 = temp_E' *temp_p1(:,ii);
                x2tEx1 = dot(temp_p1(:,ii),Ex1);
                
                %             a = Ex1(1) * Ex1(1);
                %             b = Ex1(2) * Ex1(2);
                %             c = Etx2(1) * Etx2(1);
                %             d = Etx2(2) * Etx2(2);
                
                res_array(ii,jj) = x2tEx1.^2./ sum(Ex1(1:2).^2 + Etx2(1:2).^2);
                
            end
        end
        
        
        
        
        [min_res, min_indx]= min(res_array,[],2);
        
        summed_res =  sum(min_res);
        if summed_res < best_res
            best_E_mats = E_mats;
            best_K_mats = K_mats;
            best_res = summed_res;
            best_min_res =min_res;
            best_min_res_indx =min_indx;
            %best_array = res_array;
        end
    end
    
end

end
