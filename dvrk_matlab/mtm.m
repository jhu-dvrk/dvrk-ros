classdef mtm < arm
    % Class for MTM specific features

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        state_gripper_current_subscriber;
        % publishers
        lock_orientation_publisher;
        unlock_orientation_publisher;
        % message placeholder
        geometry_msgs_Quaternion = rostype.geometry_msgs_Quaternion;
        std_msgs_Empty = rostype.std_msgs_Empty;
    end

    methods

        function self = mtm(name)
            self@arm(name);

            % ----------- subscribers
            % state gripper current
            topic = strcat(self.ros_name, '/state_gripper_current');
            self.state_gripper_current_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);

            % ----------- publishers
            topic = strcat(self.ros_name, '/lock_orientation');
            self.lock_orientation_publisher = rospublisher(topic, ...
                                                           rostype.geometry_msgs_Quaternion);
            topic = strcat(self.ros_name, '/unlock_orientation');
            self.unlock_orientation_publisher = rospublisher(topic, ...
                                                             rostype.std_msgs_Empty);

            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Empty = rosmessage(rostype.std_msgs_Empty);
            self.geometry_msgs_Quaternion = rosmessage(rostype.geometry_msgs_Quaternion);
        end


        function [position, velocity, effort, timestamp] = get_state_gripper_current(self)
            % Accessor used to retrieve the last current gripper position/effort
            position = self.state_gripper_current_subscriber.LatestMessage.Position;
            velocity = self.state_gripper_current_subscriber.LatestMessage.Velocity;
            effort = self.state_gripper_current_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.state_gripper_current_subscriber.LatestMessage.Header.Stamp);
        end


        function result = lock_orientation_as_is(self)
            position_current = self.get_position_current();
            current_orientation = position_current(1:3,1:3);
            self.lock_orientation(current_orientation);
            result = true;
        end



        function result = lock_orientation(self, ...
                                           orientation_matrix)
            if ~isreal(orientation_matrix)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': lock_orientation, input must be an array or real numbers'));
                return
            end

            if ~ismatrix(orientation_matrix)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': lock_orientation, input must be a matrix'));
               return
            end

            [nbRows, nbCols] = size(orientation_matrix);
            if (nbRows ~= 3) || (nbCols ~=3)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': lock_orientation, input must be a 3x3 matrix'));
               return
            end

            % prepare the ROS message
            % convert to ROS idiotic data type
            quaternion = rotm2quat(orientation_matrix);
            self.geometry_msgs_Quaternion.W = quaternion(1);
            self.geometry_msgs_Quaternion.X = quaternion(2);
            self.geometry_msgs_Quaternion.Y = quaternion(3);
            self.geometry_msgs_Quaternion.Z = quaternion(4);
            % send message
            send(self.lock_orientation_publisher, ...
                 self.geometry_msgs_Quaternion);
            result = true;
        end



        function result = unlock_orientation(self)
            % send message (empty)
            send(self.unlock_orientation_publisher, ...
                 self.std_msgs_Empty);
            result = true;
        end

    end % methods

end % class
