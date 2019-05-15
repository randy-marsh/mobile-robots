%% Create a Vector Field Histogram Object and Visualize Data
% This example shows how to calculate a steering direction based on input
% laser scan data.
%% Goal Position
robotGoal = [19.00   19.00];
%% init ros
rosinit('172.22.31.206')
%% create subscibers and publishers
scanSub = rossubscriber('/robot0/laser_1');
[velPub, velMsg] = rospublisher('/robot0/cmd_vel');

%% create tftree 
tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.
pause(1);
scanMsg = receive(scanSub);
%% Get robot pose at the time of sensor reading
pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

% Convert robot pose to 1x3 vector [x y yaw]
position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
    pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
robotPose = [position, orientation(1)];
%%

% Create a |VectorFieldHistogram| object.
vfh = robotics.VectorFieldHistogram('UseLidarScan',true);
% vfh.NumAngularSectors = 5
vfh.SafetyDistance = 0.5
%%
% Input laser scan data and target direction.

% %% get scan data
% scanMsg = receive(scanSub);
% scan = lidarScan(scanMsg);
% 
% 
% angles = linspace(-pi,pi,500);
% % get target Dir
% [targetDirDeg, targetDirRad] = getAngle(position, robotGoal);
% % diff relative to yawl
% targetDir = robotPose(3)- targetDirRad;
% %%
% % Compute an obstacle-free steering direction.
% steeringDir = vfh(scan,targetDir);
% % Calculate velocities
% if ~isnan(steeringDir) % If steering direction is valid
%     desiredV = 0.2;
%     w = exampleHelperComputeAngularVelocity(steeringDir, 1);
% else % Stop and search for valid directionsteerDir
%     desiredV = 0.2;
%     w = 0.0;
% end
% velMsg.Linear.X = desiredV;
% velMsg.Angular.Z = w;
% send(velPub, velMsg);
goalRadius = 0.1;
distanceToGoal = norm(robotPose(1:2) - robotGoal);

%%
% The |<docid:robotics_ref.buoofp1-1 controller>| object computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot. The controller runs at 10 Hz.
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    % Extract current location information ([X,Y]) from the current pose of the
    scanMsg = receive(scanSub);
    scan = lidarScan(scanMsg)
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
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
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    

end
rosshutdown;