function res = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
A = zeros(8*length(data(t).id),9);
    
    for i = 1:length(data(t).id)    %find large A matrix
        corner_w = getCorner(data(t).id(i));  %i is point in a specific time stamp
        corner_c = [data(t).p1(:,i)'; data(t).p2(:,i)'; data(t).p3(:,i)'; data(t).p4(:,i)'];
        
        A_tag = zeros(8,9);
        for index = 1:4
            xi = corner_w(index,1); yi = corner_w(index,2);
            xip = corner_c(index,1); yip = corner_c(index,2);
            A_tag((2*index-1):2*index,:) = [xi, yi, 1, 0, 0, 0, -xip*xi, -xip*yi, -xip;
                                            0, 0, 0, xi, yi, 1, -yip*xi, -yip*yi,-yip];
                     
        end
        A(8*i-7:8*i,:) = A_tag;
        
    end
    
    [U,S,V] = svd(A);
    h = [V(1,9),V(2,9),V(3,9);
        V(4,9),V(5,9),V(6,9);
        V(7,9),V(8,9),V(9,9)];
    h_cali = sign(h(3,3)).*h; % make sure h(3,3) is positive
    K = [311.0520,  0, 201.8724;
        0,  311.3885,  113.6210;
        0, 0, 1];
    R_withT = inv(K) * h_cali; % R_withT is [R_hat1, R_hat2, T_hat]
    R_hat = [R_withT(:,1),R_withT(:,2),cross(R_withT(:,1),R_withT(:,2))];
    [U1,~ ,V1] = svd(R_hat);
    R = U1*[1, 0, 0;     0, 1, 0;     0, 0, det(U1*V1')] * V1';
    T = R_withT(:,3) / norm(R_withT(:,1));
    
    orientation = zeros(3,1);
    orientation(1,1) = atan2(R(3,3),R(3,2)) + pi/2;
    orientation(2,1) = atan2(-1*R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    orientation(3,1) = atan2(R(2,1),R(1,1)) +pi/4;
    position = T + [-0.04; 0.0; -0.03];
    res = [position,orientation,h_cali];
end