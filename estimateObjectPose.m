function estimateObjectPose(coordinator)
% 객체를 Place 위치에 따라 분류하는 함수
    
    % 객체 관련 리스트 초기화
    coordinator.Target.Pose = [];
    coordinator.Target.RMSE = [];
    coordinator.Target.GraspPose = [];
    coordinator.Target.AccessPose = [];
    coordinator.Target.PlaceIndx=0;

    
    % 프레임 중앙에 있는 객체를 가져옴
    rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
    [bboxes, scores, labels] = detect(coordinator.DetectorModel, rgbImg, 'Threshold',0.7);      % Yolo v4
    numObjs=size(bboxes,1);
    distCenter=inf;
    for i=1:numObjs
        dx=bboxes(i,1)+bboxes(i,3)/2-coordinator.Cx;
        dy=bboxes(i,2)+bboxes(i,4)/2-coordinator.Cy;
        dist=sqrt(dx*dx+dy*dy);
        if dist<distCenter
            distCenter=dist;
            targetIndx=i;
        end
    end

    % 뎁스 이미지를 가져옴
    depthImg = readImage(coordinator.ROSinfo.depthImgSub.LatestMessage);
%     figure(1)
%     imshow(depthImg);
    
    % 카메라의 로봇 베이스 기준 좌표 계산
    camera_transf = getTransform(coordinator.ROStf, 'world', 'camera_link');
    camera_transl = [camera_transf.Transform.Translation.X camera_transf.Transform.Translation.Y camera_transf.Transform.Translation.Z];
    camera_rotation = [camera_transf.Transform.Rotation.W camera_transf.Transform.Rotation.X camera_transf.Transform.Rotation.Y camera_transf.Transform.Rotation.Z];
    Transf=trvec2tform(camera_transl)*quat2tform(camera_rotation);
    fixedRotation = eul2tform([0 pi 0],"XYZ");
    Transf = Transf*fixedRotation';


    % 뎁스 이미지로부터 객체만을 가져옴
    objX=round(bboxes(targetIndx,1));
    objY=round(bboxes(targetIndx,2));
    objWidth=round(bboxes(targetIndx,3));
    objHeight=round(bboxes(targetIndx,4));
    objLabel=labels(targetIndx);

%     objDepthImg=depthImg(objY:objY+objHeight, objX:objX+objWidth);
%     figure(2)
%     imshow(objDepthImg);
        
    % 로봇 베이스를 기준으로 객체에 대한 포인트 클라우드 생성
    objPoints=zeros((objHeight+1)*(objWidth+1),3);
    idx=1;
    for r=objY:objY+objHeight
        for c=objX:objX+objWidth
            z=depthImg(r,c);
            x=(c-coordinator.Cx)*z/coordinator.Focal;
            y=(r-coordinator.Cy)*z/coordinator.Focal;


            res=Transf*[x y z 1]';

            objPoints(idx,1)=res(1);
            objPoints(idx,2)=res(2);
            objPoints(idx,3)=res(3);
            idx=idx+1;
        end
    end
    objPC=pointCloud(objPoints);
    objPC=pcdownsample(objPC,'gridAverage',0.01);
%     figure(3)
%     pcshow(objPC);
    
    if objLabel=='bottle'
        pcTarget=coordinator.PcBottle;
        coordinator.Target.PlaceIndx=1;
    else
        pcTarget=coordinator.PcCan;
        coordinator.Target.PlaceIndx=2;
    end

    % 인식한 객체의 자세 추정(ICP 알고리즘)
    [tformsTarget,objectTransformed,rmseTarget] = pcregistericp(objPC, pcTarget, 'Metric','pointToPoint', 'Tolerance', [0.0001, 0.0001], 'MaxIterations', 1000);
    tformsTarget = invert(tformsTarget);
    rotation=tformsTarget.Rotation;
    translation=tformsTarget.Translation;
    T=[rotation(1, :), translation(1);
        rotation(2, :), translation(2);
        rotation(3, :), translation(3);
        0, 0, 0, 1];
    coordinator.Target.Pose=T;
    coordinator.Target.RMSE=rmseTarget;

%     figure(4)
%     hold on
%     quiver3(translation(1), translation(2), translation(3), rotation(1,3), rotation(2, 3), rotation(3, 3))
%     xlabel('x'); ylabel('y'); zlabel('z');
%     grid on
%     axis equal
%     legend;
%     hold off

    
    
    if rotation(3,3)>=0.7
        Amat=[coordinator.Target.Pose(1:3,1), coordinator.Target.Pose(1:3,2)];
        Pmat=Amat*((Amat'*Amat)\Amat');
        qvec=[0.7071; 0.7071; 0];
        pvec=Pmat*qvec;
        pvec=pvec/norm(pvec);
        theta=acos(dot(pvec,coordinator.Target.Pose(1:3,1)));
        if dot(cross(pvec,coordinator.Target.Pose(1:3,1)), coordinator.Target.Pose(1:3,3))>0
            theta=-theta;
        end 

        coordinator.Target.GraspPose=[coordinator.Target.Pose(:,2), coordinator.Target.Pose(:,1), -coordinator.Target.Pose(:,3), coordinator.Target.Pose(:,4)];
        coordinator.Target.GraspPose(1:3,1:3)=coordinator.Target.GraspPose(1:3,1:3)*[cos(-theta), -sin(-theta), 0; sin(-theta), cos(-theta), 0; 0, 0, 1];
        
        
        if objLabel=='bottle'
            coordinator.Target.GraspPose(3,4)=coordinator.Target.GraspPose(3,4)+0.1;
        else
            coordinator.Target.GraspPose(3,4)=coordinator.Target.GraspPose(3,4)+0.04;
        end

        if coordinator.CaptureIndx==6
            coordinator.Target.GraspPose(3,4)=objPC.ZLimits(2)-0.03;
        end

    else
        Amat=[coordinator.Target.Pose(1:3,1), coordinator.Target.Pose(1:3,2)];
        Pmat=Amat*((Amat'*Amat)\Amat');
        qvec=coordinator.Target.Pose(1:3,4)-Transf(1:3,4);
        qvec=qvec/norm(qvec);
        pvec=Pmat*qvec;
        pvec=pvec/norm(pvec);
        theta=acos(dot(pvec,coordinator.Target.Pose(1:3,1)));
        if dot(cross(pvec,coordinator.Target.Pose(1:3,1)), coordinator.Target.Pose(1:3,3))>0
            theta=-theta;
        end

        coordinator.Target.GraspPose=[coordinator.Target.Pose(:,2), coordinator.Target.Pose(:,3), coordinator.Target.Pose(:,1), coordinator.Target.Pose(:,4)];
        coordinator.Target.GraspPose(1:3,1:3)=coordinator.Target.GraspPose(1:3,1:3)*[cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    end

    coordinator.Target.AccessPose=coordinator.Target.GraspPose;
    if coordinator.CaptureIndx==5
        coordinator.Target.AccessPose(3,4)=0.3;
    elseif coordinator.CaptureIndx==6
        coordinator.Target.AccessPose(3,4)=0.2;
    else
        coordinator.Target.AccessPose(3,4)=0.4;
    end
 
    coordinator.FlowChart.estimated;
end


