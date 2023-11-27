%% ROS 통신
rosIP='192.168.58.137';
rosshutdown;
rosinit(rosIP, 11311)

%% 초기값 설정
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
RoboCupManipulation_setInitialConfig; % DO NOT MODIFY

%% Coordinator 생성 및 Flowchart 시작
currentJointConfig = homeConfiguration(robot);
coordinator = coordinatorPickPlace(robot, "gripper");

coordinator.FlowChart = FlowChartPickPlace('coordinator', coordinator);
coordinator.FlowChart.startPickPlace;