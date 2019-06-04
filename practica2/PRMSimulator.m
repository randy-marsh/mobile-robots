%% Tune the Number of Nodes
% Use the |NumNodes| property on the |PRM| object to tune the algorithm. |NumNodes| specifies 
% the number of points, or nodes, placed on the map, which the algorithm uses 
% to generate a roadmap. Using the |ConnectionDistance| property as a threshold 
% for distance, the algorithm connects all points that do not have obstacles 
% blocking the direct path between them.
% 
%
% Increasing the number of nodes can increase the efficiency of the path by 
% giving more feasible paths. However, the increased complexity increases 
% computation time. To get good coverage of the map, you might need a large 
% number of nodes. Due to the random placement of nodes, some areas of the 
% map may not have enough nodes to connect to the rest of the map. In this 
% example, you create a large and small number of nodes in a roadmap.

%%
% Load a map file and create an occupancy grid.

load('mapa_real_ajustado_limpio.mat')
mapInflated = copy(map);
robotRadius = 0.3;
inflate(mapInflated,robotRadius);
% show(mapInflated)
% 
%% init ros
rosinit('http://192.168.1.125:11311')

tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.
pause(1);


%%
% Create a simple roadmap with 250 nodes.

prm = robotics.PRM(mapInflated,250);
prm.ConnectionDistance = 1;
prm.NumNodes = 250;
%% get initial pose
scanSub = rossubscriber('/robot0/laser_1');
[velPub, velMsg] = rospublisher('/robot0/cmd_vel');
scanMsg = receive(scanSub);
% Get robot pose at the time of sensor reading
pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

% Convert robot pose to 1x3 vector [x y yaw]
position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
    pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
robotPose = [position, orientation(1)];

%%
% Calculate a simple path.

startLocation = position;
endLocation = [19 19];
path = findpath(prm,startLocation,endLocation);

%% launch pure pursuit controller
controller = robotics.PurePursuit

%%
% Use the path obtained by PRM to set the desired waypoints for the
% controller
controller.Waypoints = path;
% Set the path following controller parameters
controller.DesiredLinearVelocity = 0.5;
% Set the maximun angular velocity
controller.MaxAngularVelocity = 2;
% Set the lookahead distance 
controller.LookaheadDistance = 0.5;
% and the goal radous
goalRadius = 1;
% var to iterate
distanceToGoal = norm(position - endLocation);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    % get pose
    scanMsg = receive(scanSub);
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);

    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];

    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotPose);
    
    % move the robot
    velMsg.Linear.X = v;
    velMsg.Angular.Z = omega;
    send(velPub, velMsg);
     
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
    distanceToGoal = norm(robotPose(1:2) - endLocation);
    
    waitfor(controlRate);
    
end
% Stop therobot

velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);
rosshutdown;



