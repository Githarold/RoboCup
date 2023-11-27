function activateGripper(coordinator, action)
% 그리퍼를 동작하는 함수

       if action == 1
           % Activate gripper
            pause(1);
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd'); % rosactionclient : ROS Action에 대한 Client 생성
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.04; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal); % sendGoal : ROS Action에 대한 Goal message를 Server에 전송
            disp('Gripper closed...');
       else
           % Deactivate gripper
            pause(1);
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.0; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal);
            disp('Gripper open...');
       end
       
       pause(2);
       
       % Trigger Stateflow chart Event
       coordinator.FlowChart.activateComplete; 
end