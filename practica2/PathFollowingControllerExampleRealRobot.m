%% Path Following for a Differential Drive Robot
%% Introduction
% This example demonstrates how to control a robot to follow a desired path
% using a Robot Simulator. The example uses the Pure Pursuit path following
% controller to drive a simulated robot along a predetermined path. A desired path is a
% set of waypoints defined explicitly or computed using a path planner (refer to
% <docid:robotics_examples.example-PathPlanningExample>). The Pure Pursuit
% path following controller for a simulated differential drive robot is created and
% computes the control commands to follow a given path. The computed control commands are
% used to drive the simulated robot along the desired trajectory to
% follow the desired path based on the Pure Pursuit controller.
%
% Note: Starting in R2016b, instead of using the step method to perform the
% operation defined by the System object, you can call the object with
% arguments, as if it were a function. For example, |y = step(obj,x)| and
% |y = obj(x)| perform equivalent operations.

% Copyright 2014-2016 The MathWorks, Inc.
%% Define Waypoints
% Define a set of waypoints for the desired path for the robot

path = [14.77    10.93;
    17.41    12.13;
    18.17    13.78;
    18.27    16.68]
 

% Set the current location and the goal location of the robot as defined by the path

robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
% init ros
rosinit('http://172.29.30.176:11311')

% enable driver
pub_enable=rospublisher('/cmd_motor_state','std_msgs/Int32');
%declaración mensaje
msg_enable_motor=rosmessage(pub_enable);
%activar motores enviando enable_motor = 1
msg_enable_motor.Data=1;
send(pub_enable,msg_enable_motor);

tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.
pause(1);
scanSub = rossubscriber('/scan');
poseSub = rossubscriber('/pose');
[velPub, velMsg] = rospublisher('/cmd_vel');
% Get robot pose at the time of sensor reading
pose = receive(poseSub);

% Convert robot pose to 1x3 vector [x y yaw]
position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
    orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
robotPose = [position, orientation(1)];
robotPose(1) = robotPose(1) +14.77; 
robotPose(2) = robotPose(2) +10.93; 
robotPose(3) = robotPose(3) + 0.27;

%%
% Assume an initial robot orientation (the robot orientation is the angle
% between the robot heading and the positive X-axis, measured
% counterclockwise).
initialOrientation = 0.27

%%
% Define the current pose for the robot [x y theta]
robotCurrentPose = robotPose;

% Create a |VectorFieldHistogram| object.
vfh = robotics.VectorFieldHistogram('UseLidarScan',true);
% vfh.NumAngularSectors = 5
vfh.DistanceLimits = [0.4 1]
vfh.RobotRadius = 0.15
vfh.SafetyDistance = 0.3
% vfh.MinTurningRadius = 0.5
vfh.TargetDirectionWeight = 5
vfh.CurrentDirectionWeight = 2
vfh.PreviousDirectionWeight  = 2
vfh.HistogramThresholds = [0.5 1.2]





%% Define the Path Following Controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>| object.
controller = robotics.PurePursuit

%%
% Use the path defined above to set the desired waypoints for the
% controller
controller.Waypoints = path;

%%
% Set the path following controller parameters. The desired linear
% velocity is set to 0.3 meters/second for this example.
controller.DesiredLinearVelocity = 0.2;

%%
% The maximum angular velocity acts as a saturation limit for rotational velocity, which is
% set at 2 radians/second for this example.
controller.MaxAngularVelocity = 1;

%%
% As a general rule, the lookahead distance should be larger than the desired
% linear velocity for a smooth path. The robot might cut corners when the
% lookahead distance is large. In contrast, a small lookahead distance can
% result in an unstable path following behavior. A value of 0.5 m was chosen
% for this example.
controller.LookaheadDistance = 0.3;

%% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
% The path following controller provides input control signals for the
% robot, which the robot uses to drive itself along the desired path.
%
% Define a goal radius, which is the desired distance threshold
% between the robot's final location and the goal location. Once the robot is
% within this distance from the goal, it will stop. Also, you compute the current
% distance between the robot location and
% the goal location. This distance is continuously checked against the goal
% radius and the robot stops when this distance is less than the goal radius.
%
% Note that too small value of the goal radius may cause the robot to miss
% the goal, which may result in an unexpected behavior near the goal.
goalRadius = 1;
distanceToGoal = norm(position - robotGoal);

%%
% The |<docid:robotics_ref.buoofp1-1 controller>| object computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot. The controller runs at 10 Hz.
controlRate = robotics.Rate(20);
while( distanceToGoal > goalRadius )
    % get pose
    scanMsg = receive(scanSub);
    scan = lidarScan(scanMsg);
    pose = receive(poseSub);
    % Get robot pose at the time of sensor reading
    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
    orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    robotPose(1) = robotPose(1) +14.77; 
    robotPose(2) = robotPose(2) +10.93; 
    robotPose(3) = robotPose(3) + 0.27;


    % get target Dir
    [targetDirDeg, targetDirRad] = getAngle(robotPose(1:2), robotGoal);
    targetDir = targetDirRad - robotPose(3);
    % Compute an obstacle-free steering direction.
    steeringDir = vfh(scan,targetDir)
     % Calculate velocities
    if ~isnan(steeringDir) % If steering direction is valid
        desiredV = 0.2;
        w = exampleHelperComputeAngularVelocity(steeringDir, 1);
    else
        
        % Compute the controller outputs, i.e., the inputs to the robot
        [desiredV, w] = controller(robotPose);
    end
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    
    waitfor(controlRate);
    % Extract current location information ([X,Y]) from the current pose of the
    scanMsg = receive(scanSub);
    scan = lidarScan(scanMsg);
    pose = receive(poseSub);
    % Get robot pose at the time of sensor reading
   

    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y];
    orientation =  quat2eul([pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    robotPose(1) = robotPose(1) +14.77; 
    robotPose(2) = robotPose(2) +10.93; 
    robotPose(3) = robotPose(3) + 0.27;


    % Re-compute the distance to the goal
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

%%
% <<path_completed_path_part1.png>>
%

%%
% The simulated robot has reached the goal location using the path following
% controller along the desired path. Close simulation.
% stop robot
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);
rosshutdown

