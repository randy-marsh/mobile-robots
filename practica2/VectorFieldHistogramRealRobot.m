%% Create a Vector Field Histogram Object and Visualize Data
% This example shows how to calculate a steering direction based on input
% laser scan data.
%% Goal Position
robotGoal = [19.00   19.00];
%% init ros
rosinit('http://172.29.30.178:11311')
%% create subscibers and publishers
[velPub, velMsg] = rospublisher('/cmd_vel');
scanSub = rossubscriber('/scan');
poseSub = rossubscriber('/pose');
pose = receive(poseSub);


%% create tftree 
tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.
pause(1);
scanMsg = receive(scanSub);
% Convert robot pose to 1x3 vector [x y yaw]

position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
robotPose = [position, orientation(1)];
robotPose(1) = robotPose(1) +5; 
robotPose(2) = robotPose(2) +5; 
robotPose = [position, orientation(1)];
%%

% Create a |VectorFieldHistogram| object.
vfh = robotics.VectorFieldHistogram('UseLidarScan',true);
% vfh.NumAngularSectors = 5
vfh.DistanceLimits = [0.15 2]
vfh.RobotRadius = 0.15
vfh.SafetyDistance = 0.5
% vfh.MinTurningRadius = 0.5
vfh.TargetDirectionWeight = 5
vfh.CurrentDirectionWeight = 2
vfh.PreviousDirectionWeight  = 2
vfh.HistogramThresholds = [2 8]

goalRadius = 1;
distanceToGoal = norm(robotPose(1:2) - robotGoal);

%%
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    % Extract current location information ([X,Y]) from the current pose of the
    scanMsg = receive(scanSub);
    scan = lidarScan(scanMsg);
    pose = receive(poseSub);
    % Convert robot pose to 1x3 vector [x y yaw]
 
    position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
    orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    robotPose(1) = robotPose(1) +5; 
    robotPose(2) = robotPose(2) +5; 
    robotPose = [position, orientation(1)];
    % get target Dir
    [targetDirDeg, targetDirRad] = getAngle(robotPose(1:2), robotGoal);
    
    targetDir = targetDirRad - robotPose(3);
%     targetDir = 0;
    %%
    % Compute an obstacle-free steering direction.
    steeringDir = vfh(scan,targetDir)
    % Calculate velocities
    if ~isnan(steeringDir) % If steering direction is valid
        desiredV = 0.2;
        w = exampleHelperComputeAngularVelocity(steeringDir, 1);
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.2;
    end

    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    
    waitfor(controlRate);
    % Extract current location information ([X,Y]) from the current pose of the
    scanMsg = receive(scanSub);
    pose = receive(poseSub);
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
    orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    robotPose(1) = robotPose(1) +5; 
    robotPose(2) = robotPose(2) +5; 
    robotPose = [position, orientation(1)];
    robotPose = [position, orientation(1)];

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    

end
% stop robot
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);
rosshutdown