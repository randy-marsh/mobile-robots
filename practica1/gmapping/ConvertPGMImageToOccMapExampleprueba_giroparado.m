%% Convert PGM Image to Map
% Convert a portable graymap (|.pgm|) file containing a ROS map
% into an |OccupancyGrid| map for use in MATLAB.

%%
% Import the image using |imread|. Crop the image to the relevant area.
path = 'C:\Users\Juan\Documents\MUII\robotizados\practicas\matlabscripts\gmapping\';
file = 'prueba_giroparado';
image =  imread(strcat(path, file, '.pgm'));
imshow(image)
imageCropped = image(1625:2086, 2092:2427);
imshow(imageCropped)

%%
% PGM values are expressed from 0 to 255 as |uint8|. Normalize these values by  
% converting the cropped image to |double| and dividing each cell by 255. 
% This image shows obstacles as values close to 0. 
% Subtract the normalized image from 1 to get occupancy values 
% with 1 representing occupied space.
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;

%%
% Create the |OccupancyGrid| object using an adjusted map image. The imported map
% resolution is 20 cells per meter.
map = robotics.OccupancyGrid(imageOccupancy,20);
show(map)
save 'C:\Users\Juan\Documents\MUII\robotizados\practicas\matlabscripts\gmapping\prueba_giroparado.mat' map
