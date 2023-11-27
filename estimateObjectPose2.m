function estimateObjectPose2(coordinator)
% 객체를 Place 위치에 따라 분류하는 함수
% detection 기준 가까운 위치부터!

    cnt_loop = coordinator.cnt_loop;
    capture_pc(coordinator, cnt_loop)
    cnt_loop = cnt_loop + 1;
    coordinator.cnt_loop = cnt_loop;
    
    % 객체 관련 리스트 초기화
    coordinator.Target.Pose = [];
    coordinator.Target.RMSE = [];
    coordinator.Target.GraspPose = [];
    coordinator.Target.AccessPose = [];
    coordinator.Target.PlaceIndx=0;

    % 뎁스 이미지를 가져옴
    depthImg = readImage(coordinator.ROSinfo.depthImgSub.LatestMessage);
    
    % 카메라의 로봇 베이스 기준 좌표 계산
    camera_transf = getTransform(coordinator.ROStf, 'world', 'camera_link');
    camera_transl = [camera_transf.Transform.Translation.X camera_transf.Transform.Translation.Y camera_transf.Transform.Translation.Z];
    camera_rotation = [camera_transf.Transform.Rotation.W camera_transf.Transform.Rotation.X camera_transf.Transform.Rotation.Y camera_transf.Transform.Rotation.Z];
    Transf=trvec2tform(camera_transl)*quat2tform(camera_rotation);
    fixedRotation = eul2tform([0 pi 0],"XYZ");
    Transf = Transf*fixedRotation';

    targetIndx = 1;
    while 1
        target_objs=coordinator.Objs{targetIndx};
        bboxes=target_objs.box;
        labels = target_objs.label;
        % 뎁스 이미지로부터 객체만을 가져옴
        objX=round(bboxes(1,1));
        objY=round(bboxes(1,2));
        objWidth=round(bboxes(1,3));
        objHeight=round(bboxes(1,4));
        objLabel=labels(1);
    
%         objDepthImg=depthImg(objY:objY+objHeight, objX:objX+objWidth);
%         figure(2)
%         imshow(objDepthImg);
            
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
    %     objPC=pcdownsample(objPC,'gridAverage',0.01);
    %     figure(3)
    %     pcshow(objPC);
    
        offset = (1 - 1);
    
        bbox_vertex = [objPC.XLimits(1)-objPC.XLimits(1)*offset, objPC.XLimits(2)-objPC.XLimits(2)*offset, ...
                       objPC.YLimits(1)-objPC.YLimits(1)*offset, objPC.YLimits(2)-objPC.YLimits(2)*offset, objPC.ZLimits(1), objPC.ZLimits(2)]; %rrt하면 쓸것
    
        pcloud=coordinator.pcloud;
        pcloud_idx = find(pcloud.Location(:,1) >= bbox_vertex(1) & pcloud.Location(:,1) <= bbox_vertex(2) ...
                           & pcloud.Location(:,2) >= bbox_vertex(3) & pcloud.Location(:,2) <= bbox_vertex(4) ...
                           & pcloud.Location(:,3) >= bbox_vertex(5) & pcloud.Location(:,3) <= bbox_vertex(6));
    
        objPC = select(pcloud,pcloud_idx);

        indxPlane = find(objPC.Location(:,3) > -0.045 & objPC.Location(:,3) < 0.35 & objPC.Location(:,2) > -0.633 & objPC.Location(:,1) < +1.5); %-1.5
        objPC = select(objPC,indxPlane);
        [labels,numClusters] = pcsegdist(objPC,0.011);  %0.012
    
        num_points = 0;
        for ii=1:numClusters
            cluster_size = size(find(labels==ii));
            cluster_size = cluster_size(1);
            if cluster_size > num_points
                num_points = cluster_size;
                k = find(labels==ii)';            
            end
        end
        objPC = select(objPC,k);

        if objLabel=='bottle'
            pcTarget=coordinator.PcBottle;
            coordinator.Target.PlaceIndx=1;
        else
            pcTarget=coordinator.PcCan;
            coordinator.Target.PlaceIndx=2;
        end
    
        pcTarget = pcdenoise(pcTarget);

        objPC=pcdownsample(objPC,'gridAverage',0.01); %0.01
    
        [tformsTarget,~,rmseTarget] = pcregistericp(objPC, pcTarget, 'Metric','pointToPoint','InlierRatio',1);%, 'Tolerance', [0.0001, 0.0001], 'MaxIterations', 1000);
        tformsTarget = invert(tformsTarget);
    
        center_obj = [(objPC.XLimits(1) + objPC.XLimits(2))/2 (objPC.YLimits(1) + objPC.YLimits(2))/2 objPC.ZLimits(2)-0.015];%(objPC.ZLimits(1) + objPC.ZLimits(2))/2]
    
        tformsTarget.Translation;
        rotation=tformsTarget.Rotation;

        if rotation(3,3) >= 0.9
            translation=center_obj;
            T=[rotation(1, :), translation(1);
                rotation(2, :), translation(2);
                rotation(3, :), translation(3);
                0, 0, 0, 1];
            coordinator.Target.Pose=T;
            coordinator.Target.RMSE=rmseTarget;

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
            
            theta_x = deg2rad(45);
            rot_x =  [1 0 0 0; 0 cos(theta_x) -sin(theta_x) 0; 0 sin(theta_x) cos(theta_x) 0; 0 0 0 1];
            coordinator.Target.GraspPose = coordinator.Target.GraspPose * rot_x;

            up_pose = coordinator.Target.GraspPose;
            up_pose(3,4) = up_pose(3,4)+0.15;
            coordinator.Target.AccessPose = up_pose;

            coordinator.FlowChart.estimated;
            break
        
        else
            targetIndx = targetIndx + 1;
            if targetIndx > numel(coordinator.Objs)
                coordinator.estimation_trig=1;
                break
            end
            
        end
    end
end


