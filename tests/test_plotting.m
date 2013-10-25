
clear all;
close all
dat = load_measurements(20);


imu = dat.imu.vals;


figure(1)
subplot(311)
plot(1:length(imu), imu(4,:), 'r');
hold on;
plot(1:length(imu), imu(5,:), 'g');
hold on;
plot(1:length(imu), imu(6,:), 'b');
hold on;

subplot(312)
plot(1:length(imu), imu(1,:), 'r');
hold on;
plot(1:length(imu), imu(2,:), 'g');
hold on;
plot(1:length(imu), imu(3,:), 'b');
hold on;

subplot(313)
enc = dat.encoders.counts
plot( 1:length(enc), enc(1,:), 'r')
hold on;
plot( 1:length(enc), enc(2,:), 'g')
hold on;
plot( 1:length(enc), enc(3,:), 'b')
hold on;
plot( 1:length(enc), enc(4,:), 'y')


