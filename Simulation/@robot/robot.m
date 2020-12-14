classdef robot < handle
    properties
        sim             % Connection to coppeliasim remote api
        clientID        % ID of current simulation
        bot_ref         % Reference of robot model
        end_ref         % Reference of end effector
        
        motors          % Handler for wheels
        joints          % Handler for arm joints
        cameras         % Handler for mounted cameras
        proximity       % Handler for proximity sensor
        
        doors           % Handler for doors in simulation world
        
        frame_left      % Container for left camera image
        frame_front     % Container for front camera image
        frame_right     % Container for right camera image
        
        update_pass     % how many times images updated (for saving output)
        resolution      % camera resolution (expected to be same for all)
        
        joint_angle     % the current absolute joint angles of robot arm
        joint_position  % linear translation between revolution joints (constant)
        
        fk              % forward kinematics symbolic function
        J               % Jacobian for forward kinematics
    end
    
    methods
        
        % Class Contructor
        function self = robot()
            self.sim = remApi('remoteApi');
            self.sim.simxFinish(-1);
            self.clientID = self.sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
            disp("Client ID: " + string(self.clientID));
        end
        
        
        % Sim destructor
        function [] = destructor(self)
            self.sim.delete();
        end
        
        
        % Initialization function for all motor and camera handles
        % (PARTIALLY IMPLEMENTED)
        function [mot_ret_code, jnt_ret_code, cam_ret_code, prx_ret_code] = initialize_robot(self)

            % Initialize door controls
            door_names = ["_doorJoint_1", "_doorJoint_2", "_doorJoint_3", "_doorJoint_4", "_doorJoint_5", "_doorJoint_6", "_doorJoint_71", "_doorJoint_72"];
            self.doors = zeros(size(door_names));
            for i=1:length(self.doors)
                [~, self.doors(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(door_names(i))),self.sim.simx_opmode_blocking);
            end
            disp("Door handles: " + num2str(self.doors));
            
            
            % Initialize wheels
            wheel_names = ["rollingJoint_fl","rollingJoint_rl", "rollingJoint_rr", "rollingJoint_fr"];
            mot_ret_code = zeros(size(wheel_names));
            self.motors = zeros(size(wheel_names));
            for i=1:length(self.motors)
                [mot_ret_code(i), self.motors(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(wheel_names(i))),self.sim.simx_opmode_blocking);
            end
            disp("Wheel motor handles: " + num2str(self.motors));
            
            
            % Initialize cameras
            camera_names = ["Vision_left", "Vision_front", "Vision_right"];
            cam_ret_code = zeros(size(camera_names));
            self.cameras = zeros(size(camera_names));
            for i=1:length(self.cameras)
                [cam_ret_code(i), self.cameras(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(camera_names(i))),self.sim.simx_opmode_blocking);
                [~, self.resolution, ~] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(i), 0, self.sim.simx_opmode_streaming);
            end 
            disp("Camera handles: " + num2str(self.cameras));
            self.update_pass = 0;

            
            % Initialize proximity
            self.proximity = zeros(1, 2);
            prx_ret_code = zeros(size(self.proximity));
            
            
            % Initialize joints
            joint_names = ["youBotArmJoint0", "youBotArmJoint1", "youBotArmJoint2", "youBotArmJoint3", "youBotArmJoint4", "youBotGripperJoint1", "youBotGripperJoint2"];
            jnt_ret_code = zeros(size(joint_names));
            self.joints = zeros(size(joint_names));
            self.joint_angle = zeros(size(joint_names));

            for i=1:length(self.joints)
                [jnt_ret_code(i), self.joints(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(joint_names(i))),self.sim.simx_opmode_blocking);
                [~, self.joint_angle(i)] = self.sim.simxGetJointPosition(self.clientID, self.joints(i), self.sim.simx_opmode_blocking);
            end
            
            disp("Arm joint handles: " + num2str(self.joints));
            disp("Arm joint angles: " + num2str(self.joint_angle));
            
            % Getting link lengths
            self.joint_position = zeros(5, 3);
            [~, self.bot_ref] = self.sim.simxGetObjectHandle(self.clientID,uint8(char("bot_ref")),self.sim.simx_opmode_blocking);
            [~, self.end_ref] = self.sim.simxGetObjectHandle(self.clientID,uint8(char("end_ref")),self.sim.simx_opmode_blocking);
            disp("Robot arm reference handle: " + num2str(self.bot_ref));
            disp("End effector reference handle: " + num2str(self.end_ref));
            
            [~, self.joint_position(1, :)] = self.sim.simxGetObjectPosition(self.clientID , self.joints(1), self.bot_ref, self.sim.simx_opmode_blocking);
            [~, self.joint_position(2, :)] = self.sim.simxGetObjectPosition(self.clientID , self.joints(2), self.joints(1), self.sim.simx_opmode_blocking);
            [~, self.joint_position(3, :)] = self.sim.simxGetObjectPosition(self.clientID , self.joints(3), self.joints(2), self.sim.simx_opmode_blocking);
            [~, self.joint_position(4, :)] = self.sim.simxGetObjectPosition(self.clientID , self.joints(4), self.joints(3), self.sim.simx_opmode_blocking);
            [~, self.joint_position(5, :)] = self.sim.simxGetObjectPosition(self.clientID , self.end_ref, self.joints(4), self.sim.simx_opmode_blocking);

            % Initializing forward and inverse kinematics calculator
            [self.fk, self.J] = kinematics_symbolic(self);
            disp("Kinematic solvers initialized");
        end
        
        
        % Disconnection from robot
        function [] = terminate_robot(self)
            self.sim.simxFinish(self.clientID);
        end
        
        
        % Update images from the cameras
        function [ret_code] = update_cameras(self)
            ret_code = zeros(size(self.cameras));
            self.update_pass = self.update_pass + 1;
            
            [ret_code(1), ~ , self.frame_left] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(1), 0, self.sim.simx_opmode_buffer);
            [ret_code(2), ~ , self.frame_front] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(2), 0, self.sim.simx_opmode_buffer);
            [ret_code(3), ~ , self.frame_right] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(3), 0, self.sim.simx_opmode_buffer);
            
        end
        
        
        % Function to save images to file
        function save_images(self, directory)
            imwrite(self.frame_left,[directory, '/fl_', num2str(self.update_pass), '.jpg']);
            imwrite(self.frame_front,[directory, '/ff_', num2str(self.update_pass), '.jpg']);
            imwrite(self.frame_right,[directory, '/fr_', num2str(self.update_pass), '.jpg']);
        end
        
        
        % Update proximity data (NOT YET IMPLEMENTED)
        function [ret_code] = update_proximity(self)
            ret_code = zeros(size(self.proximity));
        end
        
        
        % Open/close door
        function [ret_code] = control_door(self, door_no, direction)
            if door_no <= 6
                [ret_code] = self.sim.simxSetJointPosition(self.clientID , self.doors(door_no), -direction*pi/2, self.sim.simx_opmode_blocking);
            else
                [ret_code] = self.sim.simxSetJointPosition(self.clientID , self.doors(door_no), direction*2, self.sim.simx_opmode_blocking);
            end
        end
        
        
        % Send velocity commands to wheel motor handlers
        function [ret_code] = set_wheel_velocity(self, wheel_velocity)

            ret_code = zeros(size(self.motors));

            for i=1:length(self.motors)
                [ret_code(i)] = self.sim.simxSetJointTargetVelocity(self.clientID , self.motors(i), wheel_velocity(i), self.sim.simx_opmode_blocking);
            end

        end
        
        
        % Send position commands to joint position handlers
        function [ret_code] = set_joint_position(self, joint_target)

            ret_code = zeros(size(self.joints));

            for i=1:length(self.joints)
                %if joint_target(i) > pi
                %    joint_target(i) = 2*pi - joint_target(i)-2*pi;
                %end
                [ret_code(i)] = self.sim.simxSetJointTargetPosition(self.clientID , self.joints(i), joint_target(i), self.sim.simx_opmode_blocking);
            end
        end
        
        
        % Update position feedback from joint position handlers
        function [ret_code] = update_joint_angle(self)

            ret_code = zeros(size(self.joints));

            for i=1:length(self.joints)
                [ret_code(i), self.joint_angle(i)] = self.sim.simxGetJointPosition(self.clientID , self.joints(i), self.sim.simx_opmode_blocking);
            end

        end
        
        
        % functions defined in seperate files
        core_routine(self)
        lfr_routine(self, destination)
        arm_routine(self, command)
        
    end
end





