%{
    A quick test script for tform etc 
%}
clear all;
dat = load_measurements(23);

%% Params
wheel_circ = 2*pi*2.5;
cnt_to_rad = 1/180;

%imu_scl = 3300/1023/pi/180;
imu_scl =  3300/1023*pi/180;
imu_bias = 370;

%% Run this shit

state = [0 0 0]; % x, y, theta
states = [state];
times = [0];
imu_ind = 1;
encoder_ind = 1;

ws = [];
vs = [];

% Compute transforms 
for i = 1:length(dat.encoders.ts)-1
    
    dt = dat.encoders.ts(i+1)-dat.encoders.ts(i);
    v = wheel_circ*cnt_to_rad*mean( dat.encoders.counts(:,i) ); 
    [~,w_ind] = min(abs(dat.imu.ts - dat.encoders.ts(i)));
    w = imu_scl*(dat.imu.vals(4,w_ind)-imu_bias);
    ws = [ws w];
    vs = [vs v];
    
    x = state(1);
    y = state(2);
    theta = state(3);
    %{
    if w ~= 0
        x_new = x - v/w*sin(theta) + v/w*sin(theta + w*dt);
        y_new = y + v/w*cos(theta) - v/w*cos(theta + w*dt);
    else
        x_new = x - v*sin(theta);
        y_new = y + v*cos(theta);
    end
    %}
    x_new = x + v*cos(theta)*dt;
    y_new = y + v*sin(theta)*dt;
    theta_new = theta + w*dt;
    
    state = [x_new y_new theta_new];
    
    times = [times dat.encoders.ts(i)];
    states = [states; state];
    
    
end



close all;
figure(1);
v = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

lidar_rots = rot(dat.hokuyo.angles);
lidar_sub = 1:25:length(dat.hokuyo.angles);
lidar_rots = lidar_rots(:,:,lidar_sub);

tfs = tforms( states );
for i = 1:20:length(tfs);
    v_tf = tfs(:,:,i)*v;
    plot( v_tf(1,1:2), v_tf(2,1:2), '-r');
    hold on;
    plot( v_tf(1,3:4), v_tf(2,3:4), '-b');
    drawnow;
    plot( states(i,1), states(i,2), '*g');
    dat.encoders.counts(:,i)
    ws(i)
    
    [~,lidar_ind] = min(abs(times(i) - dat.hokuyo.ts));
    ranges = [dat.hokuyo.ranges(lidar_sub,lidar_ind) zeros(length(lidar_sub),1)];
    % might need to flip the rotation ... :/
    lidar_dat = cell2mat(arrayfun( @(j) [ranges(j,:)*lidar_rots(:,:,j) 1]', 1:length(lidar_sub), 'UniformOutput', false ));
    
    tf_lidar_dat = tfs(:,:,i)*lidar_dat;
    plot( tf_lidar_dat(1,:), tf_lidar_dat(2,:), 'r.');
    
    
    axis equal;
    
    
    input('');
end



%{
%% Compute laser points
tfs = tforms( states );

lidar_rots = rot(dat.hokuyo.angles);
v = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

close all;
figure(1);
for j = 1000:10:length(dat.hokuyo.ts)
    lidar_ts = dat.hokuyo.ts(j);
    [~,tf_ind] = min(abs(times - lidar_ts));

    ranges = [dat.hokuyo.ranges(:,j) zeros(length(dat.hokuyo.angles),1)];
    lidar_dat = cell2mat(arrayfun( @(j) [lidar_rots(:,:,j)*ranges(j,:)'; 1], 1:length(dat.hokuyo.angles), 'UniformOutput', false ));
    
    tf_lidar_dat = eye(3)*lidar_dat;
    tfs(:,:,tf_ind)
    %tf_lidar_dat = tfs(:,:,tf_ind)*lidar_dat;
        
    %tf_lidar_dat = lidar_dat'*tfs(:,:,tf_ind);
    %tf_lidar_dat = tf_lidar_dat';
    plot( tf_lidar_dat(1,:), tf_lidar_dat(2,:), 'b.');
    hold on;
    
    tf_lidar_dat = tfs(:,:,tf_ind)*lidar_dat;
    plot( tf_lidar_dat(1,:), tf_lidar_dat(2,:), 'r.');

    v_tf = tfs(:,:,tf_ind)*v;
    plot( v_tf(1,1:2), v_tf(2,1:2), '-y', 'linewidth', 3);
    hold on;
    plot( v_tf(1,3:4), v_tf(2,3:4), '-g', 'linewidth', 3);

    axis equal;
    drawnow;
    input('');
end  
%}






