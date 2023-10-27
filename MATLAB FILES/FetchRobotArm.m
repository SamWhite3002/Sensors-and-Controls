classdef FetchRobotArm < handle
    properties
        % Publishers
        posePublisher;
        gripperPublisher;
        %moveOriginPublisher;

        % Subscribers
        motionCompleteSubscriber;
        gripCompleteSubscriber;
        %originCompleteSubscriber;

        %Messages
        poseMsg;
        gripperMsg;
        %moveOriginMsg;
    end
    
    methods
        %% Constructor
        function self = FetchRobotArm()
            % Initialize ROS and set up publishers/subscribers
            try rosinit; end
            self.SetupPublishers();
            self.SetupSubscribers();
            pause(1);
        end
        
        %% Set up ROS publishers
        function SetupPublishers(self)
            [self.posePublisher, self.poseMsg] = rospublisher('Pose', 'geometry_msgs/PoseStamped');
            [self.gripperPublisher, self.gripperMsg] = rospublisher('State', 'std_msgs/Bool');
        end
        
        %% Set up ROS subscribers
        function SetupSubscribers(self)
            self.motionCompleteSubscriber = rossubscriber('Status', 'std_msgs/Bool');
            self.gripCompleteSubscriber = rossubscriber('grip_status', 'std_msgs/Bool');
        end
        
        %% Move the robot arm to a specified block
        function MoveRobotArm(self, block)
            % Prepare the message with block information
            self.poseMsg.Pose.Position.X = block.X_base(1);
            self.poseMsg.Pose.Position.Y = block.X_base(2);
            self.poseMsg.Pose.Position.Z = block.X_base(3);
       
            % Create a Quaternion message for orientation
            orientation = rosmessage('geometry_msgs/Quaternion');
            orientation.X = block.quat(1);
            orientation.Y = block.quat(2);
            orientation.Z = block.quat(3);
            orientation.W = block.quat(4);
            self.poseMsg.Pose.Orientation = orientation;
            
            % Send the message
            send(self.posePublisher, self.poseMsg);
            
            while self.motionCompleteSubscriber.LatestMessage.Data == 0 
            end
        end
        
        %% Grip or release a block
        function GripReleaseBlock(self, grip)
            % Prepare the gripper message
            self.gripperMsg.Data = grip;
            
            % Send the gripper message
            send(self.gripperPublisher, self.gripperMsg);
            
            while self.gripCompleteSubscriber.LatestMessage.Data == 0 
            end
        end
        
        %% Pick up a block
        function PickUpBlock(self, block)
            % Create a waypoint above the block
            waypoint = block;
            waypoint.X_base(3) = waypoint.X_base(3) + 0.2;
            
            % Move to the waypoint and then to the block
            self.MoveRobotArm(waypoint);
            self.MoveRobotArm(block);
            
            % Grip the block
            self.GripReleaseBlock(true);
        end
        
        %% Place a gripped block on another block
        function PlaceGrippedBlockOn(self, grippedBlock, baseBlock)
            % Create a waypoint straight up from the gripped block
            waypoint = grippedBlock;
            
            % Move to the waypoint
            self.MoveRobotArm(waypoint);
            
            % Create a goal pose just above the base block
            goalPose = self.CalcPoseAboveBlock(baseBlock);
            
            % Create a waypoint just above the goal pose
            waypoint = goalPose;
            waypoint.Pose.Position.Z = waypoint.Pose.Position.Z + 0.2;
            
            % Move to the waypoint and then to the goal pose
            self.MoveRobotArm(waypoint);
            self.MoveRobotArm(goalPose);
            
            % Release the gripped block
            self.GripReleaseBlock(false);
            
            % Move just above the placed block
            self.MoveRobotArm(waypoint);
        end
        
        %% Calculate a position to place a block above another block
        function dropOffPose = CalcPoseAboveBlock(self, baseBlock)
            dropOffPose = baseBlock;
            dropOffPose.X_base(3) = dropOffPose.X_base(3) + 0.07;
        end
    end
end
