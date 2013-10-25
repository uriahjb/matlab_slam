%{
    Attempt to find the ground plane using from 
    Kinect data
%}

k_ind = 1000;

depth = reshape(typecast(zlibUncompress(zdepth{k_ind}),'uint16'),[640 480])';
rgb   = djpeg(jrgb{k_ind});
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

%create figure
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
T = rotz(deg2rad(10.0))*roty(deg2rad(18.0));
T(3,4) = 0.507;
Y = T*[xs;ys;zs;ones(size(xs))];

% plus 0.507

xs2 = Y(1,:);
ys2 = Y(2,:);
zs2 = Y(3,:);

%extract good values
indValid = xs > 0;
z_thresh = 0.4;
indZthresh = abs(zs2) < 0.3;
indXthresh = abs(xs2) < 2.0;

indGood = indValid & indZthresh & indXthresh;

set(hFig,'xdata',xs2(indGood),'ydata', ...
         ys2(indGood),'zdata',zs2(indGood));

     
% Set of Points ... going to turn this into RANSAC
P = Y(1:3,indGood)';
r = r(indGood);
g = g(indGood);
b = b(indGood);

% Generate sequences of points

n_samples = 100;
samples = randi(size(P,1), n_samples,3);

inlier_thresh = 0.02;

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
% Plot best normal
hold on;
plot3(subP(:,1), subP(:,2), subP(:,3), 'or', 'MarkerSize', 10)


% Plot plane normal
hold on;
normal_end = normal + centroid;
plot3([centroid(1) normal_end(1)],[centroid(2) normal_end(2)],[centroid(3) normal_end(3)], '-r', 'LineWidth', 2);

inliers = abs(bsxfun(@minus,P,centroid)*normal') < inlier_thresh;
set(inlierh, 'XData', P(inliers,1), 'YData', P(inliers,2), 'ZData', P(inliers,3));

% Plot ground plane image with colors

resolution = 0.02;
map_size = 20;
map = zeros(map_size/resolution, map_size/resolution, 3);

% pos controls position on map
pos = int32([10 10]);
pxls = int32(P(inliers,1:2)/resolution) + size(map,1)/2;
pxls = bsxfun(@plus, pxls, pos);
map_inds = sub2ind(size(map), pxls(:,1), pxls(:,2));

map_r = map(:,:,1);
map_g = map(:,:,2);
map_b = map(:,:,3);
map_r(map_inds) = r(inliers);
map_g(map_inds) = g(inliers);
map_b(map_inds) = b(inliers);
map(:,:,1) = map_r;
map(:,:,2) = map_g;
map(:,:,3) = map_b;

figure()
imshow(uint8(map))








