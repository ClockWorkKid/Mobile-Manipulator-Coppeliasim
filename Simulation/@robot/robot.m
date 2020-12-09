classdef robot < handle
    properties
        sim
        clientID
        
        motors
        joints
        cameras
        proximity
        
        frames
        resolution
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
            
            self.motors = zeros(1,4);
            self.joints = zeros(1, 6);
            self.cameras = zeros(1, 3);
            self.proximity = zeros(1, 2);

            mot_ret_code = zeros(size(self.motors));
            jnt_ret_code = zeros(size(self.joints));
            cam_ret_code = zeros(size(self.cameras));
            prx_ret_code = zeros(size(self.proximity));
            
            % Initialize wheels
            wheel_names = ["rollingJoint_fl","rollingJoint_rl", "rollingJoint_rr", "rollingJoint_fr"];
            
            for i=1:length(self.motors)
                [mot_ret_code(i), self.motors(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(wheel_names(i))),self.sim.simx_opmode_blocking);
            end
            
            disp("Wheel motor handles: " + num2str(self.motors));
            
            % Initialize cameras
            camera_names = ["Vision_front", "Vision_left", "Vision_right"];
            
            for i=1:length(self.cameras)
                [cam_ret_code(i), self.cameras(i)] = self.sim.simxGetObjectHandle(self.clientID,uint8(char(camera_names(i))),self.sim.simx_opmode_blocking);
            end
            
            [~, self.resolution, ~] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(1), 0, self.sim.simx_opmode_blocking);
            
            disp("Camera handles: " + num2str(self.cameras));
            disp("Camera resolutions: " + num2str(self.resolution));
            self.frames = zeros([length(self.cameras), self.resolution, 3]);
                 
            % Initialize proximity
            
            % Initialize joints
            
        end
        
        % Disconnection from robot
        function [] = terminate_robot(self)
            self.sim.simxFinish(self.clientID);
        end
        
        % Update images from the cameras (NOT YET IMPLEMENTED)
        function [ret_code] = update_cameras(self)
            ret_code = zeros(size(self.cameras));
            for i=1:length(self.cameras)
                [ret_code(i), ~ , self.frames(i, :, :, :)] = self.sim.simxGetVisionSensorImage2(self.clientID, self.cameras(i), 0, self.sim.simx_opmode_blocking);
            end
        end
        
        % Update proximity data (NOT YET IMPLEMENTED)
         function [ret_code] = update_proximity(self)
            ret_code = zeros(size(self.proximity));
        end
        
        % Send velocity commands to wheel motor handlers
        function [ret_code] = set_wheel_velocity(self, wheel_velocity)

            ret_code = zeros(size(self.motors));

            for i=1:length(self.motors)
                [ret_code(i)] = self.sim.simxSetJointTargetVelocity(self.clientID , self.motors(i), wheel_velocity(i), self.sim.simx_opmode_blocking);
            end

        end
        
        % Send position commands to joint position handlers (NOT YET IMPLEMENTED)
        function [ret_code] = set_joint_position(self, joint_target)

            ret_code = zeros(size(self.joint));

            for i=1:length(self.joints)
                ret_code(i) = joint_target(i);
            end

        end
        
        
        % functions defined in seperate files
        core_routine(self)
        [wheel_velocity] = lfr_routine(self, destination)
        [joint_target] = arm_routine(self, command)
        
    end
end





