function moveToJointConfiginJS(coordinator, jointConfig, tolerance)
% 로봇을 Task 공간의 목표 지점으로 움직이는 함수

    % 목표 지점에 도달했는지 체크
    ik = inverseKinematics('RigidBodyTree',coordinator.Robot, "SolverAlgorithm", "LevenbergMarquardt");
    ik.SolverParameters.AllowRandomRestart = false;
    weights = [1 1 1 1 1 1];
    initialGuess = wrapToPi(coordinator.CurrentJointConfig);
    jointFinal = wrapToPi(jointConfig);

    diffJointConfig=jointFinal-coordinator.CurrentJointConfig;
    for i=1:coordinator.NumJoints
%             if ~coordinator.JointLimit(i,1)
        if diffJointConfig(i)>pi
            diffJointConfig(i)=diffJointConfig(i)-2*pi;
        elseif diffJointConfig(i)<-pi
            diffJointConfig(i)=diffJointConfig(i)+2*pi;
        end
%             end
    end
    
    jointFinal=coordinator.CurrentJointConfig+diffJointConfig;

    isAway = checkTargetAchieved();
    
    % 목표 지점에 도달하지 않았을 경우 Task trajectory를 계산
    if isAway

        % Trajectory 시간
        timeInterval = [0;3];
        trajTimes = timeInterval(1):coordinator.TimeStep:timeInterval(end);

        % Task trajectory를 계산
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes));
        
        robotPos = repmat(coordinator.CurrentJointConfig,length(s),1) + diffJointConfig.*(s'/timeInterval(2));
        robotPos = wrapToPi(robotPos);   
        
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
            disp('isAway')
        end

        % 로봇이 동작을 멈출 때까지 대기
        isMoving = true;
        numError=0;
        while isMoving
            [isMoving] = getMovementStatus(coordinator);
            pause(1);
            disp('isMoving')
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

        diffJ=abs(jointFinal-jointCurrent);

        if all(diffJ<0.02)
            isAway=false; % goal achieved       
        end        
    end
end