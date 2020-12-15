%{

The core routine runs a while loop. This routine handles all the necessary
inputs and outputs of the robot. This routine also holds the core control
structure of the robot (what to do, when to do, how to do)

In each iteration of the core routine, image and proximity data will be
recieved from the simulator, necessary sub-processes will be called, motor
and joint commands will be sent back to the simulator

Inputs:
1. Vision sensor (Camera Images)
2. Proximity sensor

Outputs:
1. Motor Velocity/Position
2. Arm Joint angle

Subprocesses 
1. lfr_routine.m (INPUTS: destination OUTPUTS: motor)
2. arm_routine.m (INPUTS: command(pick/place) OUTPUTS: joint angles)

*subprocesses can access images/proximity data from self as necessary

%}

function [] = core_routine(self) % NOT YET IMPLEMENTED, DEMO STRUCTURE

    % demo code for going back and forth 5 times
    for i = 1:5 % change with while loop later
        
        dest_point = input('Which room? ', 's'); %Enter 'Red' to go to red room
        
        keyDest = {'Blue','Purple','Green','Cyan','Red','Olive'};
        valueDest = [1 2 3 4 5 6];
        valueReturn = [2 1 4 3 6 5];
        
        Destination = containers.Map(keyDest,valueDest);
        Return = containers.Map(keyDest, valueReturn);
        
        %arm routine here to pick up medication
        self.arm_routine("pick");
    
        %lfr routine here to go to room
        self.lfr_routine(Destination(dest_point));
        
        %arm routine here to put down medication
        self.arm_routine("place");
        
        %180 turn at the end of forward pass
        flip(self);

        %lfr routine here to return to reference point
        self.lfr_routine(Return(dest_point));
        
        %180 turn turn at the end of backward pass
        flip(self);

    end
end

