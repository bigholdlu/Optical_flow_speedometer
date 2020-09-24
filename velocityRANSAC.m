function [Vel] = velocityRANSAC(optV,optPos,Z_robot_height,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow (xy_dot)
    % optPos = Position of the features in the camera frame (c_w)
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
    prev_inlier = 0;
    
    for iter = 1:40

        Pt_i = randi([1,length(optV)],1,3); %index of three random point
        Pt_dot = [optV(Pt_i(1),:)';    optV(Pt_i(2),:)';      optV(Pt_i(3),:)'];
        Pt =     [optPos(Pt_i(1),:);     optPos(Pt_i(2),:);     optPos(Pt_i(3),:)];
        %Z_c is a single number at this frame which is height of vehicle
            optPos_1 = [optPos, ones(length(optPos),1)];
            Z = zeros(length(optPos_1),1);
            for i = 1:length(optPos_1)
                Pc = R_c2w * (Z_robot_height .* optPos_1(i,:)');
                Z(i) = Pc(3);
            end

        A_svd = get_A_svd(Pt_i, Pt, Z);

        %SVD solving the "line" from three points selected
        [U,S,V] = svd(A_svd);
        s = diag(S);
        r = 1;
        while( r < size(A_svd,2) && s(r+1) >= max(size(A_svd))*eps*s(1) )
        r = r+1;
        end
        d = U'*Pt_dot;
        x = V* ( [d(1:r)./s(1:r); zeros(6-r,1) ] ); % x is the vel we want to optmz
        Vel = x;
        
        %how many point lines with in this "line"
        
        curr_inlier = 0;
        for i = 1:length(optV)
            A_sac = get_A_sac(optPos(i,:),Z(i));
            
            if ((norm(A_sac*x - optV(i,:)'))^2) < e
                curr_inlier = curr_inlier + 1;
            end
        end
        
        if curr_inlier > prev_inlier
            Vel = x;
            prev_inlier = curr_inlier;
        end
        
        
        Vel(1:3) = R_c2w * x(1:3);
        Vel(4:6) = R_c2w * x(4:6);
        
    end

end
function res = get_A_svd(Pt_i, Pt, Z)
        A_svd = [-1, 0, Pt(1,1);    
                 0, -1, Pt(1,2); 
                 -1, 0, Pt(2,1);
                 0, -1, Pt(2,2);
                 -1, 0, Pt(3,1);
                 0, -1, Pt(3,2)];
        B_svd = [Pt(1,1)*Pt(1,2), -(1+Pt(1,1)^2), Pt(1,2);
                  1+Pt(1,2)^2, -Pt(1,1)*Pt(1,2), -Pt(1,1);
                 Pt(2,1)*Pt(2,2), -(1+Pt(2,1)^2), Pt(2,2);
                  1+Pt(2,2)^2, -Pt(2,1)*Pt(2,2), -Pt(2,1);
                 Pt(3,1)*Pt(3,2), -(1+Pt(3,1)^2), Pt(3,2);
                 1+Pt(3,2)^2, -Pt(3,1)*Pt(3,2), -Pt(3,1)];
        Z_svd = [1/(Z(Pt_i(1)));
                 1/(Z(Pt_i(1)));  
                 1/(Z(Pt_i(2)));
                 1/(Z(Pt_i(2)));  
                 1/(Z(Pt_i(3)));
                 1/(Z(Pt_i(3)))];
        res = [Z_svd.*A_svd, B_svd]; %
end

function res = get_A_sac(Pt, Z)
        A_sac = [-1, 0, Pt(1);
                0, -1, Pt(2)];
        B_sac = [Pt(1)*Pt(2), -(1+Pt(1)^2), Pt(2); 
                1+Pt(2)^2, -Pt(1)*Pt(2), -Pt(1)];
        res = [(1/Z).*A_sac, B_sac];
end

