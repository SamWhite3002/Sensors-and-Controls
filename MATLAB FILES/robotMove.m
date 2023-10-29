classdef robotMove < handle
    properties
        % Publishers
        posePublisher;
        gripperPublisher;


        % Subscribers
        motionCompleteSubscriber;
        gripCompleteSubscriber;
 

        %Messages
        poseMsg;
        gripperMsg;
       
    end
    
    methods
        %% Constructor
        function self = robotMove()
            % Initialize ROS and set up publishers/subscribers
            try rosinit; end
            self.SetupPublishers();
            self.SetupSubscribers();
            pause(1);
        end
        
        %% Set up ROS publishers
        function SetupPublishers(self)
            [self.posePublisher, self.poseMsg] = rospublisher('Pose', 'geometry_msgs/PoseStamped');
            [self.gripperPublisher, self.gripperMsg] = rospublisher('Gripe_state', 'std_msgs/Bool');
        end
        
        %% Set up ROS subscribers
        function SetupSubscribers(self)
            self.motionCompleteSubscriber = rossubscriber('Status', 'std_msgs/Bool');
            self.gripCompleteSubscriber = rossubscriber('Grip_status', 'std_msgs/Bool');
        end
        
        %% Move the robot arm to a specified block
        function MoveRobotArm(self, block)
            % Prepare the message with block information
            self.poseMsg.Pose.Position.X = block.X_base(1);
            self.poseMsg.Pose.Position.Y = block.X_base(2);
            self.poseMsg.Pose.Position.Z = block.X_base(3);
       
            % Create a Quaternion message for orientation
            self.poseMsg.Pose.Orientation.X = block.quat(1);
            self.poseMsg.Pose.Orientation.Y = block.quat(2);
            self.poseMsg.Pose.Orientation.Z = block.quat(3);
            self.poseMsg.Pose.Orientation.W = block.quat(4);
    
            % Send the message
            pause(1);
            send(self.posePublisher, self.poseMsg);
            pause(1);
            while self.motionCompleteSubscriber.LatestMessage.Data == 0 
                %disp("in motionComplete loop") %for troubleshooting
            end
        end
        
        %% Grip or release a block
        function GripReleaseBlock(self, grip)
            % Prepare the gripper message
            self.gripperMsg.Data = grip;
            
            % Send the gripper message
            pause(1);
            send(self.gripperPublisher, self.gripperMsg);
            pause(1);
            while self.gripCompleteSubscriber.LatestMessage.Data == 0 
                %disp("in gripComplete loop") %for troubleshooting
            end
        end
        
        %% Pick up a block
        function PickUpBlock(self, block)
            % Create a waypoint above the block
            wp = block;
            wp.X_base(3) = wp.X_base(3) + 0.2;
            
            % Move to the waypoint and then to the block
            self.MoveRobotArm(wp);
            self.MoveRobotArm(block);
            
            % Grip the block
            self.GripReleaseBlock(true);
        end
        
        %% Place a gripped block on another block
        function PlaceGrippedBlockOn(self, grippedBlock, baseBlock)
            % Create a waypoint straight up from the gripped block
            wp = grippedBlock;
            
            % Move to the waypoint
            self.MoveRobotArm(wp);
            
            % Create a goal pose just above the base block
            goalPose = self.aboveBlock(baseBlock);
            
            % Create a waypoint just above the goal pose
            wp = goalPose;
            wp.Pose.Position.Z = wp.Pose.Position.Z + 0.2;
            
            % Move to the waypoint and then to the goal pose
            self.MoveRobotArm(wp);
            self.MoveRobotArm(goalPose);
            
            % Release the gripped block
            self.GripReleaseBlock(false);
            
            % Move just above the placed block
            self.MoveRobotArm(wp);
        end
        
        %% Calculate a position to place a block above another block
        function dropPose = aboveBlock(self, baseBlock)
            dropPose = baseBlock;
            dropPose.X_base(3) = dropPose.X_base(3) + 0.07;
        end
    end
end
