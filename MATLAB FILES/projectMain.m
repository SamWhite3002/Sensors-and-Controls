clear all 
close all

%% Create objects of relevant classes
disp('Initializing')
camera = Cam();
disp('Camera Successful')
detect = findCube();
disp('Detect Successful')
fetch = robotMove();
disp('Arm Successful')

%% Detect Blocks location
disp('Detecting red block')
detect.DetectRedBlock(camera);
disp('Detecting green block')
detect.DetectGreenBlock(camera);

%% Robot Motion
if detect.redBlockPresent == true && detect.greenBlockPresent == true
    disp('Picking up red block')
    fetch.PickUpBlock(detect.redBlock)
    disp('Placing red block on green block')
    fetch.PlaceGrippedBlockOn(detect.redBlock, detect.greenBlock);
end  
    
