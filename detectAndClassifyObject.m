function detectAndClassifyObject(coordinator)
% 객체를 인식하고 객체 자세 정보를 알아내는 함수(Pre-trained deep learning model을 사용 : centerPoint and type)

    % 객체 리스트 초기화
    objs={};
    coordinator.Objs = {};
    coordinator.NumObjs=0;

    % 카메라의 로봇 베이스 기준 좌표 계산
    camera_transf = getTransform(coordinator.ROStf, 'world', 'camera_link');
    camera_transl = [camera_transf.Transform.Translation.X camera_transf.Transform.Translation.Y camera_transf.Transform.Translation.Z];
    camera_rotation = [camera_transf.Transform.Rotation.W camera_transf.Transform.Rotation.X camera_transf.Transform.Rotation.Y camera_transf.Transform.Rotation.Z];
    Transf=trvec2tform(camera_transl)*quat2tform(camera_rotation);
    fixedRotation = eul2tform([0 pi 0],"XYZ");
    Transf = Transf*fixedRotation';

    while 1
        % 카메라로부터 이미지를 읽어옴
        rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
        depthImg = readImage(coordinator.ROSinfo.depthImgSub.LatestMessage);
%         figure(1)
%         imshow(rgbImg);
        
        % 객체를 인식하고 Label을 시각화
        [bboxes, scores, labels] = detect(coordinator.DetectorModel, rgbImg, 'Threshold', 0.7);      % Yolo v4
        
        if ~isempty(labels)
%             labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels)); % insertObjectAnnotation : 객체에 대한 Annotation을 부여
%             figure(2)
%             imshow(labeledImg);
            coordinator.NumObjs=size(bboxes,1);
            allLabels =table(labels);
            dist=zeros(coordinator.NumObjs,1);
            for i=1:coordinator.NumObjs
                obj.box=bboxes(i,:);
                obj.label=allLabels.labels(i);
                obj.score=scores(i);
                px=round(obj.box(1)+obj.box(3)/2); 
                py=round(obj.box(2)+obj.box(4)/2);
                cz=depthImg(py, px);
                cx=(px-coordinator.Cx)*cz/coordinator.Focal;
                cy=(py-coordinator.Cy)*cz/coordinator.Focal;
                rPos=Transf*[cx cy cz 1]';
                obj.pos=[rPos(1), rPos(2)];
                dist(i)=sqrt(rPos(1)^2+rPos(2)^2);
                obj.centerPose=coordinator.CapturePose{coordinator.CaptureIndx};
                obj.centerPose(1,4)=rPos(1)-sign(obj.centerPose(1,2))*0.05;
                obj.centerPose(2,4)=rPos(2);
                objs{i}=obj;
            end
            [~, I]=sort(dist);
            for i=1:coordinator.NumObjs
                coordinator.Objs{i}=objs{I(i)};
            end
            coordinator.FlowChart.detected;
            break;
        else
            coordinator.NumDetection=coordinator.NumDetection+1;
            disp(coordinator.NumDetection);
            if coordinator.NumDetection>5
                coordinator.CaptureIndx=coordinator.CaptureIndx+1;
                coordinator.FlowChart.noDetected;
                break;
            end
        end
    end
    
end