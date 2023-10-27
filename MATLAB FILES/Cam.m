classdef Cam < handle
    properties 
        rgbImg = [];
        rgbSubscriber = [];
        depthImg = [];
        depthSubscriber = [];

                % Intrinsic Camera paramters obtained from 
        px = 320.5;                 % Principal point x
        py = 240.5;                 % Principal point y
        f = 554.254691191187;       % Focal length
        K = [554.254691191187, 0, 320.5; ...          % Camera Matrix
             0, 554.254691191187, 240.5; ...
             0, 0, 1];
    end
    
    methods
        function self = Cam()
            try rosinit; end %initialise global node and connect to ROS network
            self.subscribe();
            self.store();
        end
        
        %% Set up ROS subscribers for camera data
        function subscribe(self)
            self.rgbSubscriber = rossubscriber('/head_camera/rgb/image_raw');
            pause(5); %Pause to ensure subscribers have time to initilise
            self.depthSubscriber = rossubscriber('/head_camera/depth_registered/image_raw');
            pause(5); %Pause to ensure subscribers have time to initilise
        end
        
        %% Extract the latest RGB and Depth Image from their ROS subscriber
        function store(self)                            
            self.rgbImg = readImage(self.rgbSubscriber.LatestMessage);
            self.depthImg = readImage(self.depthSubscriber.LatestMessage);
        end
    end
end

