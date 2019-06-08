%% Mapping With Known Poses
%% Introduction
% This example shows how to create a map of the environment using range
% sensor readings if the position of the robot is known at the time of sensor
% reading. This example also shows how to use the conversion functions (such as
% |<docid:robotics_ref.buofjpw quat2eul>|) from Robotics System Toolbox(TM).
%
% This example creates a map from range sensor readings and known
% poses of the robot. For the purpose of this example, you will use a MATLAB(R)
% based simulator to drive the robot, observe the range sensor readings and
% the robot poses. You can replace the simulator with either a real robot or
% a simulated robot in the Gazebo simulator, in which case you will need some means to
% get the true position of the robot at the time of sensor reading.
%
% Prerequisites:
% <docid:robotics_examples.example-PathFollowingControllerExample Path Following for a Differential Drive Robot>,
% <docid:robotics_examples.example-ROSTransformationTreeExample Accessing the tf Transformation Tree in ROS Example>,
% <docid:robotics_examples.example-ROSPublishAndSubscribeExample Exchanging Data with ROS Publishers and Subscribers Example>

% Copyright 2016-2017 The MathWorks, Inc.

%% Initialize the Robot Simulator
% Start the ROS master in MATLAB.
% TODO change rosinit
% rosinit('http://192.168.58.128:11311','NodeHost','IP_LOCAL_MACHINE')
% Inicialización de ROS con IP del master y puerto por defecto 11311,y la dirección IP de la máquina donde se ejecuta Matlab IP_LOCAL_MACHINE
%rosinit
% home ip
% rosinit('http://192.168.1.125:11311')
% lab ip
rosinit('172.22.31.206')
%%
% Initialize the robot simulator and assign an initial pose. The simulated
% robot is a two-wheeled differential drive robot with a laser range sensor.
% 
%sim = ExampleHelperRobotSimulator('simpleMap');
%setRobotPose(sim, [2 3 -pi/2]);

% Enable ROS interface for the simulator. The simulator
% creates publishers and subscribers to send and receive data over ROS.
%enableROSInterface(sim, true);

% Increase the laser sensor resolution in the simulator to
% facilitate map building.
%sim.LaserSensor.NumReadings = 50;

%%
% <<simulator_start_image.png>>
%

%% Setup ROS Interface
% Create ROS publishers and subscribers to communicate with the simulator.
% Create |<docid:robotics_ref.bupf5_j_9 rossubscriber>| for receiving laser
% sensor data from the simulator.
%%
%scanSub = rossubscriber('scan');
scanSub = rossubscriber('/robot0/laser_1');
%%
% For building the map, the robot will drive around the map and collect
% sensor data. Create |<docid:robotics_ref.bupf5_j_6 rospublisher>| to send
% velocity commands to the robot.
%% TODO change this
%[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
%%
% The MATLAB simulator publishes the position of the robot with respect to
% the map origin as a transformation on the topic |/tf|, which is used as the
% source for the ground truth robot pose in this example. The simulator
% uses |map| and |robot_base| as frame names in this transformation.
%
% Create a ROS transformation tree object using
% |<docid:robotics_ref.bupf5_j_12 rostf>| function. For building a map, it is
% essential that robot pose and laser sensor reading correspond to the
% same time. The transformation tree allows you to find pose of the robot
% at the time the laser sensor reading was observed.
tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.
pause(1);


%% Create a Path Controller
% The robot will have to drive around the entire map to collect laser
% sensor data and build a complete map. Assign a path with waypoints that
% cover the entire map.



%%
% <<simulator_with_path.png>>
%

%%
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>|
% object.

%%
% Set the controller rate to run at 10 Hz.
controlRate = robotics.Rate(10);
%% Define an Empty Map
% Define a map with high resolution using |<docid:robotics_ref.bvaw60t-1 robotics.OccupancyGrid>|
% to capture sensor readings. This creates a map with 14 m X 13 m size and
% a resolution of 20 cells per meter.
map = robotics.OccupancyGrid(15,14.9,20);
%%
% Visualize the map in the figure window.
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');


%%
% <<default_map.png>>
%

%%
% The following while loop will build the map of the environment while
% driving the robot. The following steps are performed:
%
% * First, you receive the laser scan data using the |scanSub| subscriber.
% Use the |<docid:robotics_ref.buqbijb getTransform>| function with the
% time stamp on |scan| message to get the transformation between the
% |map| and |robot_base| frames at the time of the sensor reading.
%
% * Get the robot position and orientation from the transformation. The
% robot orientation is the Yaw rotation around the Z-axis of the robot. You can
% get the Yaw rotation by converting the quaternion to euler angles
% using |<docid:robotics_ref.buofjpw quat2eul>|.
%
% * Pre-process laser scan data. The simulator returns NaN ranges for laser rays
% that do not hit any obstacle within the maximum range. Replace the NaN
% ranges by maximum range value.
%
% * Insert the laser scan observation using the |<docid:robotics_ref.bvaw7o8-1 insertRay>|
% method on the occupancy grid |map|.
%
% * Compute the linear and angular velocity commands using the |controller|
% object to drive the robot.
%
% * Visualize the map after every 50 updates.
%% create a controller to stop adquisitions
updateCounter = 1;
h = uicontrol('Style', 'Pushbutton', 'String', 'Stop', ...
              'Callback', 'delete(gcbo)');
while ishandle(h)
    % Receive a new laser sensor reading
    scanMsg = receive(scanSub);
     
    
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot0', scanMsg.Header.Stamp, 'Timeout', 2);
    
    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    
    % Extract the laser scan
    %% change this with VM data
    try

    scan = lidarScan(scanMsg);
    catch
    warning('Problem using lidarScan');
    continue
    end

    ranges = scan.Ranges;
    ranges(isnan(ranges)) = scanMsg.RangeMax;
    scan2 = removeInvalidData(scan,'RangeLimits',[scanMsg.RangeMin scanMsg.RangeMax]);
    modScan = lidarScan(ranges, scan.Angles);
    
    % Insert the laser range observation in the map
    %insertRay(map, robotPose, modScan, scanMsg.RangeMax
    insertRay(map, robotPose, modScan, 8)
    
    % Compute the linear and angular velocity of the robot and publish it
    % to drive the robot.
    %% change this
    %[v, w] = controller(robotPose);
    %velMsg.Linear.X = v;
    %velMsg.Angular.Z = w;
    %send(velPub, velMsg);
    
    % Visualize the map after every 50th update.
    if ~mod(updateCounter,50)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    
    % Update the counter and distance to goal
    updateCounter = updateCounter+1;
%    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    % Wait for control rate to ensure 10 Hz rate
    waitfor(controlRate);
end

%%
% Display the final map, which has incorporated all the sensor readings.
show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');

%%
% <<final_map.png>>
%%
% <<simulator_end_image.png>>

%% Shutdown ROS Network
% Shut down the ROS master and delete the global node.
rosshutdown

%% See Also
%
% * <docid:robotics_examples.example-PathPlanningExample Path Planning in Environments of Different Complexity>
% * <docid:robotics_examples.example-TurtleBotMonteCarloLocalizationExample Localize TurtleBot using Monte Carlo Localization>

displayEndOfDemoMessage(mfilename)
