%% Create a Vector Field Histogram Object and Visualize Data
% This example shows how to calculate a steering direction based on input laser scan data.
%% init ros
rosinit('172.22.31.206')
%% create subscibers
scanSub = rossubscriber('/robot0/laser_1');
%%
% Create a |VectorFieldHistogram| object.
 vfh = robotics.VectorFieldHistogram('UseLidarScan',true);
 
%%
% Input laser scan data and target direction.

%% get scan data
scanMsg = receive(scanSub);
scan = lidarScan(scanMsg);


angles = linspace(-pi,pi,500);
targetDir = 0;
 
%%
% Compute an obstacle-free steering direction.
steeringDir = vfh(scan,targetDir)

%%
% Visualize the |VectorFieldHistogram| computation.
h = figure;
set(h,'Position',[50 50 800 400])
show(vfh)
rosshutdown;