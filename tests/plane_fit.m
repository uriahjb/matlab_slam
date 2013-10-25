close all

% Generate set of points
P = randn(100,2)*[2 0;3 0;-1 2]'; 
P = P + randn(size(P))/3;
P = bsxfun(@plus, P, [2 1 1]);

A = bsxfun(@minus, P, mean(P));

% Find best fitting plane for all points
[U,S,V] = svd(A); 
normal = V(:,end); 

% Plot points
plot3(P(:,1), P(:,2), P(:,3), 'b.')

% Plot centroid
hold on;
centroid = mean(P);
plot3(centroid(1), centroid(2), centroid(3), 'r.');

% Plot plane normal
normal_end = normal + centroid';
plot3([centroid(1) normal_end(1)],[centroid(2) normal_end(2)],[centroid(3) normal_end(3)], '-r', 'LineWidth', 2);
axis equal;
grid on;

% Calculate error of each point from the plane
point_errors = A*normal;