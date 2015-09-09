classdef robot < handle
    
    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        robot_name
        robot_state_subscriber
        robot_state_publisher
        position_cartesian_desired_subscriber
        position_joint_desired_subscriber
        position_goal_joint_publisher
    end
    
    % values set by this class, can be read by others
    properties (SetAccess = protected)
        robot_state
        position_cartesian_desired
        position_joint_desired
    end
    
    methods
        
        function self = robot(name)
            self.robot_name = name;
            
            % ----------- subscribers 
            % state
            topic = strcat('/dvrk/', self.robot_name, '/robot_state');
            self.robot_state_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_String);
            self.robot_state_subscriber.NewMessageFcn = ...
                @(sub, data)self.robot_state_callback(sub, data);

            % position cartesian desired
            self.position_cartesian_desired = [];
            topic = strcat('/dvrk/', self.robot_name, '/position_cartesian_desired');
            self.position_cartesian_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_Pose);
            self.position_cartesian_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_cartesian_desired_callback(sub, data);
            
            % position joint desired
            self.position_joint_desired = [];
            topic = strcat('/dvrk/', self.robot_name, '/position_joint_desired');
            self.position_joint_desired_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);
            self.position_joint_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_joint_desired_callback(sub, data);
            
            % ----------- publishers
            % state
            topic = strcat('/dvrk/', self.robot_name, '/set_robot_state');
            self.robot_state_publisher = rospublisher(topic, rostype.std_msgs_String);

            % position goal joint
            topic = strcat('/dvrk/', self.robot_name, '/set_position_goal_joint');
            self.position_goal_joint_publisher = rospublisher(topic, rostype.sensor_msgs_JointState);
        end
        
        function delete(self)
           % hack to disable callbacks from subscribers
           % there might be a better way to remove the subscriber itself
           self.robot_state_subscriber.NewMessageFcn = @(a, b, c)[];
           self.position_cartesian_desired_subscriber.NewMessageFcn = @(a, b, c)[];
           self.position_joint_desired_subscriber.NewMessageFcn = @(a, b, c)[];
        end
        
        function robot_state_callback(self, subscriber, data)
            self.robot_state = data.Data;
        end
        
        function position_cartesian_desired_callback(self, subscriber, pose)
            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Position.X, pose.Position.Y, pose.Position.Z]);
            orientation = quat2tform([pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z]);
            % combine position and orientation
            self.position_cartesian_desired = position * orientation;
        end
        
        function position_joint_desired_callback(self, subscriber, jointState)
            self.position_joint_desired = jointState.Position;
        end
        
        function set_state(self, state_as_string)
            message = rosmessage(self.robot_state_publisher);
            message.Data = state_as_string;
            send(self.robot_state_publisher, message);
        end
        
        function home(self)
            self.set_state('Home');
            message = rosmessage(self.robot_state_publisher);
            message.Data = 'Home';
            send(self.robot_state_publisher, message);
        end
        
        function delta_joint_move_single(self, value, index)
            self.set_state('DVRK_POSITION_GOAL_JOINT');
            jointState = rosmessage(self.position_goal_joint_publisher);
            jointState.Position = self.position_joint_desired;
            jointState.Position(index) = jointState.Position(index) + value;
            send(self.position_goal_joint_publisher, jointState);
        end
        
        function delta_joint_move_list(self, value, index)
            if isfloat(value) && isfloat(index) && length(value) == length(index)
                self.set_state('DVRK_POSITION_GOAL_JOINT');
                jointState = rosmessage(self.position_goal_joint_publisher);
                jointState.Position = self.position_joint_desired;
                for i = 1:length(value)
                    jointState.Position(index(i)) = jointState.Position(index(i)) + value(i);
                end
                send(self.position_goal_joint_publisher, jointState);
            else
                disp('Parameters must be arrays of the same length')
            end
           
        end
        
        function joint_move_single(self, value, index)
            self.set_state('DVRK_POSITION_GOAL_JOINT');
            jointState = rosmessage(self.position_goal_joint_publisher);
            jointState.Position = self.position_joint_desired;
            jointState.Position(index) = value;
            send(self.position_goal_joint_publisher, jointState);
        end
        
        function joint_move_list(self, value, index)
            if isfloat(value) && isfloat(index) && length(value) == length(index)
                self.set_state('DVRK_POSITION_GOAL_JOINT');
                jointState = rosmessage(self.position_goal_joint_publisher);
                jointState.Position = self.position_joint_desired;
                for i = 1:length(value)
                    jointState.Position(index(i)) = value(i);
                end
                send(self.position_goal_joint_publisher, jointState);
            end
        end
        
       % function open_gripper(self)
       %     self.set_state('DVRK_POSITION_GOAL_JOINT');
       %     jointState = rosmessage(self.position_goal_joint_publisher);
       %     jointState.Position = self.position_joint_desired;
       %     jointState.Position(7) = pi/4;
       %     send(self.position_goal_joint_publisher, jointState);
       % end
        
       % function close_gripper(self)
       %     self.set_state('DVRK_POSITION_GOAL_JOINT');
       %     jointState = rosmessage(self.position_goal_joint_publisher);
       %     jointState.Position = self.position_joint_desired;
       %     jointState.Position(7) = 0.0;
       %     send(self.position_goal_joint_publisher, jointState);
       % end
        
        function delta_cartesian_move(self,value,index)
            
        end 
    end
end