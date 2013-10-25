function world = update_ground( ind, state, kinect, lidar, world, t0 )
    %% Get Kinect frame   
    [~,k_ind] = min(abs((lidar.ts(ind)+t0) - kinect.depth_ts));
    disp(k_ind)
    depth = reshape(typecast(zlibUncompress(kinect.zdepth{k_ind}),'uint16'),[640 480])';
    rgb   = djpeg(kinect.jrgb{k_ind});
       
    % this could probably be done with reshape, ... meh
    r = rgb(:,:,1);
    r = r(:);
    g = rgb(:,:,2);
    g = g(:);
    b = rgb(:,:,3);
    b = b(:);

    [yis zis] = meshgrid((1:640),(1:480));
    yis = yis(:)';
    zis = zis(:)';
    
    %convert from uint16 to double
    disp2 = double(depth(:))';
    
    %get the x values (forward distance)
    %slightly different from ROS values
    xs = 1.03 ./ (-0.00304 .* disp2 + 3.31);

    %some calibration parameters from ROS package
    fx = 585.05108211;
    fy = 585.05108211;
    cx = 315.83800193;
    cy = 242.94140713;

    %calculate y and z values
    ys = -(yis-cx) ./ fx .* xs;
    zs = -(zis-cy) ./ fy .* xs;

    %transform based on the sensor pitch
    % rotz is how we will control rotation
    theta = state(3);
    T = rotz(theta)*roty(deg2rad(18.0));
    T(3,4) = 0.507;
    Y = T*[xs;ys;zs;ones(size(xs))];
    
    % plus 0.507

    %% Take 3d points and use RANSAC to get the best fit plane
    xs2 = Y(1,:);
    ys2 = Y(2,:);
    zs2 = Y(3,:);

    %extract good values
    indValid = xs > 0;
    z_thresh = 0.4;
    indZthresh = abs(zs2) < 0.3;
    indXthresh = abs(xs2) < 2.0;
    indYthresh = abs(ys2) < 2.0;

    indGood = indValid & indZthresh & indXthresh & indYthresh;

    % Set of Points ... going to turn this into RANSAC
    P = Y(1:3,indGood)';
    r = r(indGood);
    g = g(indGood);
    b = b(indGood);

    % Generate sequences of points
    n_samples = 25;
    samples = randi(size(P,1), n_samples,3);

    inlier_thresh = 0.03;

    n_inliers = zeros(n_samples,1);
    normals = zeros(n_samples,3);

    for j = 1:n_samples
        subP = P(samples(j,:),:);
        centroid = mean(subP);
        A = bsxfun(@minus, subP, centroid);
        [U,S,V] = svd(A); 
        normal = V(:,end);
        normal(3) = abs(normal(3)); % always have a positive normal
        inliers = abs(bsxfun(@minus,P,centroid)*normal) < inlier_thresh;

        n_inliers(j) = sum(inliers);
        normals(j,:) = normal;

    end

    [~,ind] = max(n_inliers);

    subP = P(samples(ind,:),:);    
    centroid = mean(subP);
    normal = normals(ind,:);
    
    %% update map_rgb

    %resolution = 0.02;
    %map_size = 20;
    resolution = world.resolution;
    map_size = world.size;
    map = world.map_rgb;
    %map = zeros(map_size/resolution, map_size/resolution, 3);

    % pos controls position on map
    %pos = int32([10 10]);
    %pos = int32(state(1:2));
    
    pos = int32(1/world.resolution*state(1:2)) + int32(world.center);
    
    pxls = int32(P(inliers,1:2)/resolution);
    pxls = bsxfun(@plus, pxls, pos);
    map_inds = sub2ind(size(map), pxls(:,1), pxls(:,2));

    map_r = world.map_rgb(:,:,1);
    map_g = world.map_rgb(:,:,2);
    map_b = world.map_rgb(:,:,3);
    map_r(map_inds) = r(inliers);
    map_g(map_inds) = g(inliers);
    map_b(map_inds) = b(inliers);
    
    map_r(world.map > log(0.9/0.1)) = 0;
    map_g(world.map > log(0.9/0.1)) = 255;
    map_b(world.map > log(0.9/0.1)) = 0;
    
    world.map_rgb(:,:,1) = map_r;
    world.map_rgb(:,:,2) = map_g;
    world.map_rgb(:,:,3) = map_b;
    
    
    figure(3), clf(gcf);
    hFig = plot3(0,0,0,'b.');
    axis([-5 5 -5 5 -5 5]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

    inlierh = line();
    set(inlierh, 'XData', [], 'YData', [], 'ZData', []);
    set(inlierh, 'LineStyle', 'None');
    set(inlierh, 'Marker', '*');
    set(inlierh, 'Color', 'r');
    
    set(hFig,'xdata',xs2(indGood),'ydata', ...
         ys2(indGood),'zdata',zs2(indGood));



end
    