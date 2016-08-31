classdef mtm < arm
    % Class for MTM specific features

    % only this class methods can view/modify
    properties (SetAccess = private)
        % publishers
        lock_orientation_publisher
        unlock_orientation_publisher
    end

    methods

        function self = mtm(name)
            self@arm(name);

            % ----------- publishers
            topic = strcat(self.ros_name, '/lock_orientation');
            self.lock_orientation_publisher = rospublisher(topic, ...
                                                           rostype.geometry_msgs_Quaternion);
            topic = strcat(self.ros_name, '/unlock_orientation');
            self.unlock_orientation_publisher = rospublisher(topic, ...
                                                             rostype.std_msgs_Empty);
        end



        function result = lock_orientation_as_is(self)
            current_orientation = self.position_current(1:3,1:3);
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
            orientation_message = rosmessage(self.lock_orientation_publisher);
            % convert to ROS idiotic data type
            quaternion = rotm2quat(orientation_matrix);
            orientation_message.W = quaternion(1);
            orientation_message.X = quaternion(2);
            orientation_message.Y = quaternion(3);
            orientation_message.Z = quaternion(4);
            % send message
            send(self.lock_orientation_publisher, ...
                 orientation_message);
            result = true;
        end



        function result = unlock_orientation(self)
            unlock_message = rosmessage(self.unlock_orientation_publisher);
            % send message (empty)
            send(self.unlock_orientation_publisher, ...
                 unlock_message);
            result = true;
        end

    end % methods

end % class
