function capture_pc(coordinator, cnt_loop)
    pointCloudSub = rossubscriber('/camera/depth/points');
    pcsub = receive(pointCloudSub);
    pcsub = readXYZ(pcsub);
    pcsub = double(rmmissing(pcsub));
    pcsub = pointCloud(pcsub);

    camera_transf = getTransform(coordinator.ROStf, 'world', 'camera_link'); 
    camera_transl = [camera_transf.Transform.Translation.X camera_transf.Transform.Translation.Y camera_transf.Transform.Translation.Z];
    camera_rotation = [camera_transf.Transform.Rotation.W camera_transf.Transform.Rotation.X camera_transf.Transform.Rotation.Y camera_transf.Transform.Rotation.Z];
    Transf=trvec2tform(camera_transl)*quat2tform(camera_rotation);
    fixedRotation = eul2tform([0 pi 0],"XYZ");
    Transf = Transf*fixedRotation';

    trans = Transf(1:3,4)';
    rot = Transf(1:3,1:3);

    ideal_rot = [0 -1 0; -1 0 0; 0 0 -1];
    base_rot = rot\ideal_rot;

    theta_x = asin(base_rot(3,2)); 
    rot_x =  [1 0 0 ; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];

    theta_y = asin(base_rot(1,3));
    rot_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];


    theta_z = asin(base_rot(2,1));
    rot_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
    
    
    tform = rigid3d(rot_x*rot_y*rot_z,[0 0 0]);
    pcsub = pctransform(pcsub,tform);

    
    rot = Transf(1:3,1:3)*[1 0 0 ; 0 cos(theta_x) -sin(theta_x) ; 0 sin(theta_x) cos(theta_x)]*rot_y*rot_z;
    tform = rigid3d(rot,trans);
    ptCloudWorld = pctransform(pcsub,tform);

    if cnt_loop == 1
        coordinator.pcloud = ptCloudWorld;
    else
        coordinator.pcloud = pcmerge(ptCloudWorld, coordinator.pcloud, 0.004);  
    end

end