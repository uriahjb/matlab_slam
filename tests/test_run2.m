%{
    Test motion model and eventually lidar plotting 
%}

%% Load in data
clear all;
dat = load_measurements(23);

lidar = dat.hokuyo;
imu = dat.imu;
encoders = dat.encoders;

%% Params
wheel_circ = 2*pi*0.165;
cnt_to_rad = 1/360;
cnt_to_vel = wheel_circ*cnt_to_rad;
cnt_scls = [8.0];

%imu_scl =  3300/1023/pi/180;
imu_scls = [1050]/1023*pi/180
imu_bias = 370;

cfg = struct();
cfg.imu_scl = 1050/1023*pi/180;
cfg.imu_bias = 370;
cfg.cnt_to_vel = 8.0*wheel_circ*cnt_to_rad;

for cnt_ind = 1:length(cnt_scls)
    cnt_scl = cnt_scls(cnt_ind)
for scl = 1:length(imu_scls)
    imu_scl = imu_scls(scl)

%% For plotting
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';
lidar_rots = rot(dat.hokuyo.angles);
num_scans = length(lidar.angles);


%% Running test

state = [0 0 0];

vs = [];
ws = [];
thetas = [];

imu_ind = 1;
encoder_ind = 1;

figure()
title(num2str(imu_scl))

for ind = 1:length(lidar.ts)-100
    
    % Plot state and laser scan using current position
    
    if mod(ind,50) == 0
        tf = tforms(state);

        v_tf = tforms(state)*vec;
        plot( v_tf(1,1:2), v_tf(2,1:2), '-y');
        hold on;
        plot( v_tf(1,3:4), v_tf(2,3:4), '-g');
        hold on;

        %{
        ranges = [lidar.ranges(:,ind) zeros(num_scans,1)];
        lidar_dat = cell2mat(arrayfun( @(j) [lidar_rots(:,:,j)*ranges(j,:)'; 1], 1:num_scans, 'UniformOutput', false ));
        tf_lidar_dat = tf*lidar_dat;
        %}
        
        tf_lidar_dat = tform_scan( state, lidar.ranges(:,ind), lidar_rots);
        
        plot( tf_lidar_dat(1,:), tf_lidar_dat(2,:), 'r.');
        hold on;

        drawnow;
        axis equal;
    end
    
    %{
    ts0 = lidar.ts(ind); 
    ts = lidar.ts(ind+1);
     
    % Update state
    dt = ts - ts0;
    
    imu_i0 = find( imu.ts > ts0 );
    imu_i1 = find( imu.ts > ts );
    
    enc_i0 = find( encoders.ts > ts0 );
    enc_i1 = find( encoders.ts > ts );
    
    counts = mean(sum( encoders.counts(:,enc_i0(1):enc_i1(1)) ));    
    
    w = imu_scl*(mean(imu.vals(4,imu_i0(1):imu_i1(1)))-imu_bias);
    vel = cnt_scl*cnt_to_vel*counts;
    %}
    
    [vel, w, dt] = get_vel( ind, dat, cfg );
    
    ws = [ws w];
    vs = [vs vel];
    thetas = [thetas state(3)];
    
    state = update_motion( state, vel, w, dt );
end
fname = [num2str(imu_scl) '_' num2str(cnt_scl)]; 
title(fname)
print(gcf, '-dpng', ['figures/' fname])
    
end
end
%{
figure(2)
subplot(311)
plot(1:length(vs), vs, '*')
subplot(312)
plot(1:length(ws), ws, '*')
subplot(313)
plot(1:length(thetas), thetas, '*')
%}
