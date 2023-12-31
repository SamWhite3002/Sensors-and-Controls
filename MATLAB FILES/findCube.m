classdef findCube < handle
    properties
        blockWidth = 0.06;
        redBlock;
        greenBlock;
        redBlockPresent = false;
        greenBlockPresent = false;
             
    end
    methods
        %% Constructor
        function self = findCube()
        end
        
        %% Detect Blocks
        function DetectRedBlock(self, camera)
            % Get colour channel values of RGB image
            red = camera.rgbImg(:,:,1);
            green = camera.rgbImg(:,:,2);
            blue = camera.rgbImg(:,:,3);                       
            %Cycle through the image to find pixels where Red
            counter = 0;
            redPoints = [];
            y = camera.rgbSubscriber.LatestMessage.Height;
            x = camera.rgbSubscriber.LatestMessage.Width;
            for i=1:y
                for j=1:x
                    if red(i,j) > 128 && green(i,j) < 128 && blue(i,j) < 128 
                        counter = counter+1;
                        redPoints(counter,:) = [i,j];       
                    end
                end
            end
      
            % Average the position of the redPoints to find the middle        
            redPoints;
            pointR = [0,0];
            pointR = round(mean(redPoints));
            
            if isnan(pointR) == true
                self.redBlockPresent = false;
                self.redBlock.u = NaN;
                self.redBlock.v = NaN;
                disp("ERROR: NO RED BLOCK")
            else 
                self.redBlockPresent = true;
                disp("Red Block Found")
                self.redBlock.u = pointR(2);
                self.redBlock.v = pointR(1);
         
                self.redBlock.X_cam = self.DetermineBlocksLocation(camera, self.redBlock.u, self.redBlock.v);
                self.redBlock.X_base = self.TransformCameraToBase(self.redBlock.X_cam);
                self.redBlock.quat = self.determineBlocksRotation(camera, self.redBlock.u, self.redBlock.v);
            end 
            end
   
        function DetectGreenBlock(self, camera)
            % Get colour channel values of RGB image
            red = camera.rgbImg(:,:,1);
            green = camera.rgbImg(:,:,2);
            blue = camera.rgbImg(:,:,3);                       
            %Cycle through the image to find pixels where Red
            counter = 0;
            greenPoints = []; 
            y = camera.rgbSubscriber.LatestMessage.Height;
            x = camera.rgbSubscriber.LatestMessage.Width;
            for i=1:y
                for j=1:x
                    if red(i,j) < 128 && blue(i,j) < 128 && green(i,j) > 128    
                        counter = counter+1;
                        greenPoints(counter,:) = [i,j];
                       
                    end
                end
            end
            
            % Average the position of the redPoints to find the middle        
            greenPoints;
            pointG = [0,0];
            pointG = round(mean(greenPoints));

            if isnan(pointG) == true
                self.greenBlockPresent = false;
                self.greenBlock.u = NaN;
                self.greenBlock.v = NaN;
                disp("ERROR: NO GREEN BLOCK")
            else 
                self.greenBlockPresent = true;
                disp("Green Block Found")
                self.greenBlock.u = pointG(2);
                self.greenBlock.v = pointG(1);
         
                self.greenBlock.X_cam = self.DetermineBlocksLocation(camera, self.greenBlock.u, self.greenBlock.v);
                self.greenBlock.X_base = self.TransformCameraToBase(self.greenBlock.X_cam);
                self.greenBlock.quat = self.determineBlocksRotation(camera, self.greenBlock.u, self.greenBlock.v);
            end 
          
        end
        %% Determine pick up location
        function X_cam = DetermineBlocksLocation(self, camera, u, v)
            % Find Z value of point (add half the size of the block)
            Z = camera.depthImg(v,u)+self.blockWidth/2;
            % Camera intrinsic properties matrix
            K = camera.K;
            % extrinsic properties
            x = [u*Z; ...
                 v*Z; ...
                 Z];
            % Calculate the objects position in the camera's reference frame
            X_cam = inv(K)*x;
        end
        
        %% Transfer point in camera coordinate to base
        function X_base = TransformCameraToBase(self, X_cam)
            % Create a transformation tree and ROS node
            tftree = rostf;
            node = ros.Node('/Transform/Points/Camera_to_base');
            
            % Wait for the transform between 2 frames
            waitForTransform(tftree, 'base_link', 'head_camera_rgb_optical_frame', 5);
            
            % Define the point in the camera's coordinate frame
            pointCam = rosmessage('geometry_msgs/PointStamped');
            pointCam.Header.FrameId = 'head_camera_rgb_optical_frame';
            pointCam.Point.X = X_cam(1);
            pointCam.Point.Y = X_cam(2);
            pointCam.Point.Z = X_cam(3);
            
            % Transform the point to the base link
            pointBase = transform(tftree, 'base_link', pointCam);
            X_base = [pointBase.Point.X; pointBase.Point.Y; pointBase.Point.Z];
        end
    
        %% Determine the rotation of the block
        function quat = determineBlocksRotation(self, camera, u, v)
        % Initialize counters and variables
        counter = 2;
        h = 0;
        b = 0;
    
        % Calculate the initial left and right depths
        dl = camera.depthImg(v, u - counter);
        dr = camera.depthImg(v, u + counter);
    
        % Calculate the initial height and the left and right X coordinates
        left = self.DetermineBlocksLocation(camera, u - counter, v);
        xl = left(1);
        right = self.DetermineBlocksLocation(camera, u + counter, v);
        xr = right(1);
    
        % Calculate the base angle theta
        h = dr - dl;
        b = abs(xr - xl);
        theta = atan(h / b);
    
        % Initialize slope with the base angle
        slope = theta;
    
        % Move along the block to its edges while keeping the slope within the threshold
        while (slope > theta - 0.01) && (slope < theta + 0.01)
            counter = counter + 1;
    
            % Update left and right depths
            dl = camera.depthImg(v, u - counter);
            dr = camera.depthImg(v, u + counter);
    
            % Calculate the new height and the left and right X coordinates
            left = self.DetermineBlocksLocation(camera, u - counter, v);
            xl = left(1);
            right = self.DetermineBlocksLocation(camera, u + counter, v);
            xr = right(1);
    
            % Calculate the new base
            h = dr - dl;
            b = abs(xr - xl);
    
            % Update the slope
            slope = atan(h / b);
        end
    
        % Recalculate the slope by taking the second last points (corners of the block)
        dl = camera.depthImg(v, u - counter + 2);
        dr = camera.depthImg(v, u + counter - 2);
    
        % Calculate the new height and the left and right X coordinates
        left = self.DetermineBlocksLocation(camera, u - counter + 2, v);
        xl = left(1);
        right = self.DetermineBlocksLocation(camera, u + counter - 2, v);
        xr = right(1);
    
        % Calculate the final base angle theta
        h = dr - dl;
        b = abs(xr - xl);
        theta = atan(h / b);
    
        % Convert to quaternion
        quat = eul2quat([0, -pi/2, theta], 'ZYX'); % Default is ZYX
    end

   
    end
end

