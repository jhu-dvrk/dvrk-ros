classdef mtm < dvrk.arm
    % Class for MTM specific features

    % only this class methods can view/modify
    properties (Access = protected)
        % publishers
        lock_orientation_publisher;
        unlock_orientation_publisher;
        % message placeholder
        geometry_msgs_Quaternion = rostype.geometry_msgs_Quaternion;
        std_msgs_Empty = rostype.std_msgs_Empty;
    end

    properties (SetAccess = immutable)
        gripper;
    end

    methods

        function self = mtm(name)
            self@dvrk.arm(name);
            self.gripper = dvrk.mtm_gripper(strcat(name, '/gripper'));
            % ----------- publishers
            topic = strcat(self.ros_namespace, '/lock_orientation');
            self.lock_orientation_publisher = rospublisher(topic, ...
                                                           rostype.geometry_msgs_Quaternion);
            topic = strcat(self.ros_namespace, '/unlock_orientation');
            self.unlock_orientation_publisher = rospublisher(topic, ...
                                                             rostype.std_msgs_Empty);

            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Empty = rosmessage(rostype.std_msgs_Empty);
            self.geometry_msgs_Quaternion = rosmessage(rostype.geometry_msgs_Quaternion);
        end

        function delete(self)
            delete(self.gripper);
        end

        function result = lock_orientation_as_is(self)
            position_current = self.setpoint_cp();
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
