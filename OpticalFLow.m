%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);
v = VideoReader('sfstreet.mp4');
length = 34*30;
width = 246;
vel = zeros(1,length);
%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

for n = 2:length%length(sampledData)
    I = read(v,n-1);
    I = I(:,1:width);
    imshow(I);
    %% Initalize Loop load images
    %c_w = corner(sampledData(n-1).img);
    c_w = detectHarrisFeatures(I);
    strong = selectStrongest(c_w,5);
    %% Initalize the tracker to the last frame.
    
    pointTracker = vision.PointTracker;
    initialize(pointTracker,  strong.Location,  I);%sampledData(n-1).img);
    
    
    %% Find the location of the next points;
    %videoFrame = sampledData(n).img;
    videoFrame = read(v,n);
    videoFrame = videoFrame(:,1:width);
    [c_w_tracked,validity] = pointTracker(videoFrame);
    %imshow(videoFrame);
    displace = c_w_tracked-strong.Location;
    %velocity = displace/(sampledData(n).t-sampledData(n-1).t);
    vel(n-1) = displace(1,1);
    %% Calculate velocity
    % Use a for loop

%     %% Calculate Height
% 
%     Pose = estimatePose(sampledData,n);
%     Z_robot_height = Pose(3,1);
%     Z = zeros(length(xy_dot),1);
%     R = Pose(:,3:5);
%     for i = 1:length(xy_dot)
%         Position = R * [c_w_tracked(i,:),0]';
%         Z(i) = Position(3);
%     end
%     %% RANSAC    
%     % Write your own RANSAC implementation in the file velocityRANSAC
%     Vel = velocityRANSAC(xy_dot,c_w_tracked, Z_robot_height ,R ,0.9);
%     
%     %% Thereshold outputs into a range.
%     % Not necessary
%     
%     %% Fix the linear velocity
%     % Change the frame of the computed velocity to world frame
%     
%     %% ADD SOME LOW PASS FILTER CODE
%     % Not neceessary but recommended 
%     %estimatedV(:,n) = Vel;
%     
%     %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
%     estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
%     % Structure of the Vector Vel should be as follows:
end
plot(1:length,vel)
%plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
