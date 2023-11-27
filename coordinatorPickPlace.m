classdef coordinatorPickPlace < handle
% Stateflow를 실행하고, Pick and place 데이터를 저장하는 클래스

    properties         
        FlowChart

        % Robot
        Robot
        EndEffector
        CurrentJointConfig
        CurrentTaskConfig
        HomeTaskConfig
        NumJoints
        JointLimit
        
        % ROS
        ROSinfo
        ROStf
        Focal=346.59975954875006;
        Cx=240.5;
        Cy=135.5;

        % Trajectory
        TimeStep
        
        % Detection
        CapturePose
        CaptureIndx=1;
        DetectorModel;
        NumDetection=0;
        Objs
        NumObjs=0;
        ObjIndx=1;

        % Estimation
        PcBottle
        PcCan
        Target

        % PickPlace
        PlacePose
        RedObjPose

        %ysh
        pcloud
        cnt_loop = 1;
        detect_trig = 0;
        DetectorModel2
        estimation_trig = 0;

        yellowpose
        inbox
        boxpose
    end
    
    methods
        % 생성자
        function obj = coordinatorPickPlace(robot, endEffector)
            % ROS
            obj.ROSinfo.jointsSub = rossubscriber('/my_gen3/joint_states');                 % rossubscriber : 해당 Topic의 Subscriber 생성
            obj.ROSinfo.configClient = rossvcclient('/gazebo/set_model_configuration');     % rossvcclient : Service를 제공하는 Server와 Client를 연결
            obj.ROSinfo.gazeboJointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'}; % joint names of robot model in GAZEBO
            obj.ROSinfo.controllerStateSub = rossubscriber('/my_gen3/gen3_joint_trajectory_controller/state');
            obj.ROSinfo.rgbImgSub = rossubscriber('/camera/color/image_raw');
            obj.ROSinfo.depthImgSub = rossubscriber('/camera/depth/image_raw');
            obj.ROStf=rostf;

            % GAZEBO physics
            physicsClient = rossvcclient('gazebo/unpause_physics');
            physicsResp = call(physicsClient,'Timeout',3);                                  % call : Service의 Server를 Call하여 응답을 받아옴
            
            % Robot
            obj.Robot = robot;   
            obj.CurrentJointConfig = getCurrentJointConfig(obj);
            obj.EndEffector = endEffector;
            obj.CurrentTaskConfig = getTransform(obj.Robot, obj.CurrentJointConfig, obj.EndEffector);
            obj.TimeStep = 0.01; % used by Motion Planner
            obj.NumJoints = numel(obj.CurrentJointConfig);   
            obj.HomeTaskConfig=getTransform(obj.Robot, obj.CurrentJointConfig, obj.EndEffector);
            obj.JointLimit=[0 -inf inf; 1 -2.2 2.2; 0 -inf inf; 1 -2.5656 2.5656; 0 -inf inf; 1 -2.05 2.05; 0 -inf inf];

            % Detection
            detector = load('yolov4_model.mat');
            obj.DetectorModel = detector.detector;

            detector = load('yolov4_model2.mat');
            obj.DetectorModel2 = detector.net;

            obj.CapturePose={[0 1 0 -0.05; 1 0 0 0.5; 0  0 -1 0.4; 0 0 0 1.0];
                            [0 1 0 0.3; 1 0 0 0; 0 0 -1 0.4; 0 0 0 1.0];
                            [0 1 0 0.2; 1 0 0 -0.5; 0 0 -1 0.4; 0 0 0 1.0];
                            [0 1 0 -0.05; 1 0 0 -0.5; 0 0 -1 0.4; 0 0 0 1.0];
                            [0 1 0 0.2; 1 0 0 0.32; 0 0 -1 0.4; 0 0 0 1.0];
                            [0 1 0 0.35; 1 0 0 -0.25; 0 0 -1 0.4; 0 0 0 1.0];
                            [];
                            [0 0.9962 0.0872 0.62; 1 0 0 0.14; 0 0.0872 -0.9962 0.34; 0 0 0 1]; %yellow
                            [0 0.866 0.5 0.23; 1.0 0 0 -0.5; 0 0.5 -0.866 0.53; 0 0 0 1]; %inbox
                            };
           
            % Estimation
            stlTarget = stlread('BottlePoints.stl');
            pcTarget = pointCloud(stlTarget.Points);
            obj.PcBottle=pcdownsample(pcTarget,'gridAverage',0.01);
            stlTarget=stlread('CanPoints.stl');
            pcTarget = pointCloud(stlTarget.Points);
            obj.PcCan=pcdownsample(pcTarget,'gridAverage',0.01);

            % PickPlace
            obj.PlacePose={[1, 0, 0, -0.35; 0, -1, 0, -0.45; 0, 0, -1, 0.4; 0, 0, 0, 1];
                            [-1, 0, 0, -0.35; 0, 1, 0, 0.45; 0, 0, -1, 0.4; 0, 0, 0, 1]};
            
            obj.RedObjPose={[-pi/5 pi/2 0 0 0 0 0];
                        [-pi/5 pi/2 0 0 0 5*pi/9 0];
                        [-pi/5 pi/2 0 0 0 17*pi/48 0];
                        [-pi/5 pi/2+pi/36 0 0 0 17*pi/48-3*pi/72 0]};
            

            %seok
            obj.yellowpose={[0 1 0 0.77; 0.9962 0 -0.0872 0.48; -0.0872 0 -0.9962 0.38; 0 0 0 1];
                [0 0.9397 0.342 0.715; 1 0 0 0.38; 0 0.342 -0.9397 0.26; 0 0 0 1];
                [0 1 0 0.8; 0.8192 0 0.5736 -0.08; 0.5736 0 -0.8192 0.35; 0 0 0 1];
                [0 0.9962 0.0872 0.62; 1 0 0 0.14; 0 0.0872 -0.9962 0.34; 0 0 0 1]};

            obj.inbox={[0 1 0 0.33; 0.4226 0 -0.9063 0.25; -0.9063 0 -0.4226 0.48; 0 0 0 1];
                [0 1 0 0.52; 0.2588 0 -0.9659 0.25; -0.9659 0 -0.2588 0.38; 0 0 0 1];
                [0 1 0 0.68; 0.2588 0 -0.9659 0.4; -0.9659 0 -0.2588 0.28; 0 0 0 1];
                [0 1 0 0.75; 0.866 0 -0.5 -0.165; -0.5 0 -0.866 0.45; 0 0 0 1];
                [0 0.9962 0.0872 0.85; 1 0 0 -0.7; 0 0.0872 -0.9962 0.5; 0 0 0 1];
                [0 0.9962 0.0872 0.615; 1 0 0 -0.7; 0 0.0872 -0.9962 0.5; 0 0 0 1];
                [0 0.866 0.5 0.23; 1.0 0 0 -0.5; 0 0.5 -0.866 0.53; 0 0 0 1];
                };

        end
        
        % 현재 로봇의 Joint 정보를 받아오는 함수
        function JConfig = getCurrentJointConfig(obj)
            jMsg = receive(obj.ROSinfo.jointsSub);      % receive : 새로운 ROS 메세지를 받아옴
            JConfig =  jMsg.Position(2:8)';
        end
        
        % 현재 로봇의 동작 여부를 받아오는 함수
        function isMoving = getMovementStatus(obj)
            statusMsg = receive(obj.ROSinfo.controllerStateSub);
            velocities = statusMsg.Actual.Velocities;
            if all(velocities<0.03)
                isMoving = 0;
            else
                isMoving = 1;
            end
        end
        
        % stateflow chart를 제거하는 함수
        function delete(obj)
            delete(obj.FlowChart)
        end
            
    end
  
end

