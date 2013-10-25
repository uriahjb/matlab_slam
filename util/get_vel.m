%{
    Given lidar index get velocity and angular velocity
%}

function [ vel, w, dt ] = get_vel( ind, dat, cfg )
    imu_scl = cfg.imu_scl;
    imu_bias = cfg.imu_bias;
    cnt_to_vel = cfg.cnt_to_vel;
    
    imu = dat.imu;
    lidar = dat.hokuyo;
    encoders = dat.encoders;
    
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
    vel = cnt_to_vel*counts;
    
end