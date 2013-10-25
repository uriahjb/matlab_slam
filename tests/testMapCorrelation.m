global MAP

hokuyoFile = 'data/Hokuyo20.mat';

load(hokuyoFile);

MAP.res   = 0.05; %meters

MAP.xmin  = -40;  %meters
MAP.ymin  = -40;
MAP.xmax  =  40;
MAP.ymax  =  40;


%dimensions of the map
MAP.sizex  = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey  = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);

MAP.map = zeros(MAP.sizex,MAP.sizey,'int8');


%assuming initial pose of x=0,y=0,yaw=0, put the first scan into the map
%also, assume that roll and pitch are 0 (not true in general - use IMU!)


%make the origin of the robot's frame at its geometrical center

%sensor to body transform
Tsensor = trans([0.1 0 0])*rotz(0)*roty(0)*rotx(0);

%transform for the imu reading (assuming zero for this example)
Timu = rotz(0)*roty(0)*rotx(0);

%body to world transform (initially, one can assume it's zero)
Tpose   = trans([0 0 0]);

%full transform from lidar frame to world frame
T = Tpose*Timu*Tsensor;

%xy position in the sensor frame
xs0 = (Hokuyo0.ranges(:,1).*cos(Hokuyo0.angles))';
ys0 = (Hokuyo0.ranges(:,1).*sin(Hokuyo0.angles))';

%convert to body frame using initial transformation
X = [xs0;ys0;zeros(size(xs0)); ones(size(xs0))];
Y=T*X;

%transformed xs and ys
xs1 = Y(1,:);
ys1 = Y(2,:);

%convert from meters to cells
xis = ceil((xs1 - MAP.xmin) ./ MAP.res);
yis = ceil((ys1 - MAP.ymin) ./ MAP.res);

%check the indices and populate the map
indGood = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));
MAP.map(inds) = 100;

%compute correlation
x_im = MAP.xmin:MAP.res:MAP.xmax; %x-positions of each pixel of the map
y_im = MAP.ymin:MAP.res:MAP.ymax; %y-positions of each pixel of the map

x_range = -1:0.05:1;
y_range = -1:0.05:1;

tic
c = map_correlation(MAP.map,x_im,y_im,Y(1:3,:),x_range,y_range);
toc

%plot original lidar points
figure(1);
plot(xs1,ys1,'.')

%plot map
figure(2);
imagesc(MAP.map);

%plot correlation
figure(3);
surf(c)
