function moveToTaskConfiginTS(coordinator, taskConfig, tolerance)
% 로봇을 Task 공간의 목표 지점으로 움직이는 함수

    % 목표 지점에 도달했는지 체크
    anglesTarget = rotm2eul(taskConfig(1:3,1:3),'XYZ');
    poseTarget = [taskConfig(1:3,4); anglesTarget']; 
    isAway = checkTargetAchieved(); 
    
    % 목표 지점에 도달하지 않았을 경우 Task trajectory를 계산
    if isAway
        ik = inverseKinematics('RigidBodyTree',coordinator.Robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        % 로봇의 초기 Task 공간 위치 및 자세
        taskInit = coordinator.CurrentTaskConfig;

        % 목표 Task 공간 위치 및 자세
        taskFinal = taskConfig;

        % Trajectory 시간
        timeInterval = [0;3];   % 3
        trajTimes = timeInterval(1):coordinator.TimeStep:timeInterval(end);

        % Task trajectory를 계산
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes));
        [T, ~, ~] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes, 'TimeScaling',[s;sd;sdd]/timeInterval(end));

        % Task trajectory에 대응되는 Joint 공간의 값을 계산(역기구학)
        robotPos = zeros(size(T,3),coordinator.NumJoints);
        initialGuess = wrapToPi(coordinator.CurrentJointConfig);
        for i=1:size(T,3)            
            robotPos(i,:) = ik(coordinator.EndEffector,T(:,:,i),weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %% 로봇 구동을 위해 Joint 속도 및 가속도를 계산
        disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = coordinator.TimeStep;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,coordinator.NumJoints);robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,coordinator.NumJoints);robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    

        %% JointTrajectoryController에 의해 원하는 Trajectory를 패키징하고 전송함
        disp('Done sampling trajectory, now packaging...')
        [trajAct,trajGoal] = rosactionclient('/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');  % rosactionclient : ROS Action에 대한 Client 생성
        packageJointTrajectory(trajGoal,coordinator.ROSinfo.gazeboJointNames,q,qd,qdd,trajTimes);

        disp('Done packaging trajectory, now sending...')
        sendGoal(trajAct,trajGoal)      % sendGoal : ROS Action에 대한 Goal message를 Server에 전송

        % 로봇이 목적 지점에 도달할 때까지 대기
        pause(1.0);        % this pause is needed to start asking for movement status
        isAway = true;
        while isAway
            isAway = checkTargetAchieved();
            pause(1);
        end

        % 로봇이 동작을 멈출 때까지 대기
        isMoving = true;
        numError=0;
        while isMoving
            [isMoving] = getMovementStatus(coordinator);
            pause(1);
            if numError>=20
                break;
            end
            numError=numError+1;
        end            
    end

    % 현재 로봇의 Configuration을 업데이트
    coordinator.CurrentJointConfig = getCurrentJointConfig(coordinator);
    coordinator.CurrentTaskConfig = getTransform(coordinator.Robot, coordinator.CurrentJointConfig, coordinator.EndEffector); 

    % Trigger Stateflow chart Event
    coordinator.FlowChart.moveComplete; 

    % 로봇이 목표 지점에 도달했는지 체크하는 함수
    function isAway = checkTargetAchieved()
        isAway = true;
        jointCurrent = getCurrentJointConfig(coordinator);
        taskCurrent = getTransform(coordinator.Robot, jointCurrent, coordinator.EndEffector);
        anglesCurrent = rotm2eul(taskCurrent(1:3,1:3), 'XYZ');
        poseCurrent =  [taskCurrent(1:3,4);anglesCurrent'];
        diffCurrent = abs([poseTarget(1:3)-poseCurrent(1:3); angdiff(poseCurrent(4:6),poseTarget(4:6))]);
        if all(diffCurrent<max(0.05,tolerance))       
            isAway=false; % goal achieved       
        end        
    end
end