function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
   res = zeros(2,4);
    if id<36
        res(:,4)= [0.304 * mod(id,12);
            0.304 * fix(id/12)];
    end
    if (id>=36)&&(id<72)
        res(:,4)= [0.304 * mod(id,12);
            0.304 * fix(id/12) + (0.178-0.152)];
    end
    if (id>=72)
        res(:,4)= [0.304 * mod(id,12);
            0.304 * fix(id/12) + 2*(0.178-0.152)];
    end
    res(:,1) = [res(1,4)+0.152;
                res(2,4)];
    res(:,3) = [res(1,4);
                res(2,4)+0.152];    
    res(:,2) = [res(1,4)+0.152;
                res(2,4)+0.152];    
    res = res';
    
    
end