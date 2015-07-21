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
            self.position_cartesian_desired = rostype.geometry_msgs_Pose;
            topic = strcat('/dvrk/', self.robot_name, '/position_cartesian_desired');
            self.position_cartesian_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_Pose);
            self.position_cartesian_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_cartesian_desired_callback(sub, data);
            
            % position joint desired
            self.position_joint_desired = rostype.sensor_msgs_JointState;
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
        
        function robot_state_callback(self, subscriber, data)
            self.robot_state = data.Data;
        end
        
        function position_cartesian_desired_callback(self, subscriber, pose)
            quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
            self.position_cartesian_desired = quat2rotm(quat);
            % disp(pose)
        end
        
        function position_joint_desired_callback(self, subscriber, jointState)
            self.position_joint_desired = jointState.Position;
            % disp(pose)
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
            jointState.Position
            self.position_joint_desired
            send(self.position_goal_joint_publisher, jointState);
        end
    end
end