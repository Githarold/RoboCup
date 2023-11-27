function msg = packageJointTrajectory(msg,jointNames,q,qd,qdd,trajTimes)
% 원하는 Task trajectory의 위치, 속도, 가속도를 ros_control이 요구하는 ROS 메세지 형태로 패키징하는 함수

    % Initialize values
    N = numel(trajTimes);
    numJoints = size(q,1);
    zeroVals = zeros(numJoints,1);
    
    % Assign joint names to ROS message
    msg.Trajectory.JointNames = jointNames;
    
    % 각 Joint에 제약 조건을 부여
    for idx = 1:numJoints
        msg.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance');
        msg.GoalTolerance(idx).Name = jointNames{idx};
        msg.GoalTolerance(idx).Position = 0;
        msg.GoalTolerance(idx).Velocity = 0.1;
        msg.GoalTolerance(idx).Acceleration = 0.1;
    end
    
    % Loop through waypoints and fill in their data
    trajPts(N) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    for idx = 1:N
        m = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        m.TimeFromStart = rosduration(trajTimes(idx));
        m.Positions = q(:,idx);
        m.Velocities = qd(:,idx);
        m.Accelerations = qdd(:,idx);
        m.Effort = zeroVals;
        trajPts(idx) = m;
        % Uncomment below to display packaging progress
%         if mod(idx,round(N/10))==0
%            disp(['Packing waypoint ' num2str(idx) ' of ' num2str(N) '...']); 
%         end
    end 
    msg.Trajectory.Points = trajPts;    
end
