
%% Load in data
dat = load_measurements(21);

cfg.gyro_scl = 1050/1023*pi/180;
cfg.gyro_bias = [370 374 376];
cfg.accel_bias = [512.5 500.5 606.5];
cfg.accel_scl = 1.0;

imu = dat.imu;

%% Plot raw_imu data
figure(1)
subplot(211)
title('Accel vs Time');
plot( imu.ts, imu.vals(1,:), imu.ts, imu.vals(2,:), imu.ts, imu.vals(3,:) )
subplot(212)
title('Gyro vs Time');
plot( imu.ts, imu.vals(4,:), imu.ts, imu.vals(5,:), imu.ts, imu.vals(6,:) )

%% Solve for accel calibration matrix

n_vals = 100; 
[U,S,V] = svd(imu.vals(1:3,1:n_vals)*repmat([0 0 1],n_vals,1))
accel_calib = inv(U*V');

%{
You can't do this for angular rates, since [0 0 0] doesn't provide 
enough information

truth = normrnd(repmat([0 0 0],n_vals,1),repmat([0.05 0.05 0.05],n_vals,1));
[U,S,V] = svd(imu.vals(4:6,1:n_vals)*truth)
gyro_calib = inv(U*V');

%}


accels = accel_calib*imu.vals(1:3,:);

%accels = bsxfun(@minus, imu.vals(1:3,:), cfg.accel_bias');
%accels = accels.*cfg.accel_scl;

angular_rates = bsxfun(@minus, imu.vals(4:6,:), cfg.gyro_bias');
angular_rates = angular_rates.*cfg.gyro_scl;

%% Plot edited data

figure(2)
subplot(211)
title('Accel vs Time');
plot( imu.ts, accels(1,:), imu.ts, accels(2,:), imu.ts, accels(3,:) )
subplot(212)
title('Gyro vs Time');
plot( imu.ts, angular_rates(1,:), imu.ts, angular_rates(2,:), imu.ts, angular_rates(3,:) )

%% Plot just accel based orientation estimate over time
orientation = [0 0 0 1];
gvec = [0 0 1];

gvecs = [];
for j = 1:length(imu.ts)
    accel_estimate = accels(1:3,j);
    accel_estimate = accel_estimate/norm(accel_estimate);
    accel_estimate
    gvecs = [gvecs accel_estimate];
end

%{
    set(gvech, 'XData', [0 accel_estimate(1)], 'YData', [0 accel_estimate(2)], 'ZData', [0 accel_estimate(3)]);
    drawnow
%}

rad_errs = qdiff_rad([zeros(1,length(gvecs)); gvecs], repmat([0 0 0 1],length(gvecs),1)');

figure(3)
plot( imu.ts, rad2deg(rad_errs), '-b' );
title('Accel orientation error from gravity in degrees');

%% Use gyro to estimate orientation
imu_ts = imu.ts;

qws = [];
for j = 1:length(imu_ts)
    w = [0; angular_rates(2:3,j)];
    if j == length(imu_ts)
        qw = qdelta( w, imu_ts(j) );
    else
        qw = qdelta( w, imu_ts(j+1)-imu_ts(j) );
    end
    qws = [qws qw];
end

q0 = [0 0 0 1];
qs = [];
gvecs_gyro = [];
for j = 1:length(qws)
    q0 = qmult( q0,qws(:,j) );    
    qs = [qs q0];
    gvecs_gyro = [gvecs_gyro qrotate(q0, gvec')];
end

rad_errs_gyro = qdiff_rad( gvecs_gyro, repmat([0 0 0 1],length(gvecs),1)');

figure(4)
plot( imu.ts, rad2deg(rad_errs_gyro), '-b' );
title('Gyro orientation error from gravity in degrees');

%% Now lets run a slerp based complementary orientation filter on the data and see how it goes
qs_complementary = [];
gvecs_filt = [];
qprev = [0 0 0 1]';
for j = 1:length(qws)
        
    % This is a fairly naive implementation, there might be a more
    % intelligent way to ensure to drive the error to zero
    beta = 0.02;
    qnew = qmult( qprev, qws(:,j) );   
    qprev = slerp( qnew, [0; gvecs(:,j)], beta );
    % There is a thought here that we could also "low pass" the output so
    % that it is a running average between the current state update and the
    % previous one, but i'm not sure what the right way to do this is,
    % perhaps only on the qws or something like that ... 
    %qprev = slerp( qnew, qprev, alpha );   
    if any(isnan(q0))
        error('nans :/');
    end
    qs_complementary = [qs_complementary qprev];
    gvecs_filt = [gvecs_filt qrotate(qprev, gvec')];
end
rad_errs_filt = qdiff_rad( gvecs_filt, repmat([0 0 0 1],length(gvecs),1)');

figure(5)
plot( imu.ts, rad2deg(rad_errs_filt), '-b' );
title('Filtered orientation error from gravity in degrees');



%% Plot live orientation estimate
%{
figure(5)
clf()
gvech = line();
set(gvech, 'XData', [], 'YData', [], 'ZData', []);
set(gvech, 'Color', 'r');
set(gvech, 'LineWidth', 2);
axis equal
%}



