function [] = debug(self)

%[frame_front, frame_left, frame_right] = check_current_position(self);


while true
    wheel_velocity = input('Input the four wheel velocities: ');
    for i=1:10
        self.set_wheel_velocity(wheel_velocity);
    end
    self.set_wheel_velocity([0,0,0,0]);
    %[frame_front, frame_left, frame_right] = check_current_position(self);
%     disp('front')
%     %disp(self.frame_front(:,:,3))
%     disp(frame_front(:,:,3))
%     disp('left')
%     disp(frame_left(:,:,3))
%     disp('right')
%     disp(frame_right(:,:,3))
end

end