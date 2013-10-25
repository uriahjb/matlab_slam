%{
    Load in measurement set
%}
function dat = load_measurements( data_num )
    dat = struct();
    enc = load(['data/Encoders' num2str(data_num) '.mat']);
    hokuyo = load(['data/Hokuyo' num2str(data_num) '.mat']);
    imu = load(['data/imuRaw' num2str(data_num) '.mat']);
    
    dat.encoders = enc.Encoders;
    dat.hokuyo = hokuyo.Hokuyo0;
    dat.imu = imu;
    
    t0 = min([dat.encoders.ts(1) dat.hokuyo.ts(1) dat.imu.ts(1)]);
    dat.encoders.ts = dat.encoders.ts - t0;
    dat.hokuyo.ts = dat.hokuyo.ts - t0;
    dat.imu.ts = dat.imu.ts - t0;
    dat.t0 = t0;
end