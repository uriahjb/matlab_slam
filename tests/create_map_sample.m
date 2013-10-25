%{ 
    Just using encoder data produce a map
%}

%% Load in data
dat = load_measurements(20);

%% Configuration Parameters
wheel_circ = 2*pi*0.165;
cnt_to_rad = 1/360;

cfg = struct();
cfg.imu_scl = 1050/1023*pi/180;
cfg.imu_bias = 370;
cfg.cnt_to_vel = 8.0*wheel_circ*cnt_to_rad;

p_hit = 0.55;
p_miss = 1 - p_hit;
cfg.log_hit = log(p_hit/p_miss);
cfg.confidence_thresh = log(0.9999/0.0001);

%% For plotting
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

lidar = dat.hokuyo;
lidar_rots = rot(lidar.angles);
num_scans = length(lidar.angles);

ranges_norm = [ones(length(lidar.angles),1) zeros(length(lidar.angles),1)];
lidar_norm = cell2mat(arrayfun( @(j) lidar_rots(:,:,j)*ranges_norm(j,:)', 1:length(lidar.angles), 'UniformOutput', false ));

%% Init
state = [0 0 0];

resolution = 0.05;
world_size = 75;

map_size = [world_size world_size]./resolution; % width in cells
map_center = map_size./2;
%world_size = 10; % meters
unknown = log(0.5/0.5);
map = ones(map_size)*unknown; % initialize map with uncertain

%% Want to draw particles 
figure(1)

range = [0 1.0];
scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);
im = imshow(scl*(map+cfg.confidence_thresh), 'Border', 'tight');
set(im, 'clipping', 'off');


ax = get(gcf, 'CurrentAxes');
set(ax, 'YLimMode', 'auto');
set(ax, 'XLimMode', 'auto');
set(ax, 'PlotBoxAspectRatioMode', 'auto');


ph = line();
set(ph, 'XData', [], 'YData', []);
set(ph, 'LineStyle', 'None');
set(ph, 'Marker', 's');
set(ph, 'Color', 'r' );

trajh = line();
set(trajh, 'XData', [], 'YData', []);
set(trajh, 'Color', 'r' );
traj = int32([]);

samplesh = line();
set(samplesh, 'XData', [], 'YData', []);
set(samplesh, 'Color', 'r');
set(samplesh, 'Marker', '*');
set(samplesh, 'LineStyle', 'None');

bsampleh = line();
set(bsampleh, 'XData', [], 'YData', []);
set(bsampleh, 'Color', 'g');
set(bsampleh, 'Marker', 'o');
set(bsampleh, 'LineStyle', 'None');

laserh = line();
set(laserh, 'XData', [], 'YData', []);
set(laserh, 'Color', 'r');
set(laserh, 'Marker', '.');
set(laserh, 'LineStyle', 'None');

search_range = resolution;
n_samples = 150;
%resample_dist = [0.1 0.1 0.0];
vel_sigma = 0.1;
w_sigma = 0.3;

ts0 = lidar.ts(1);
tic()


%samples = bsxfun(@times, 2*(rand(n_samples,3)-0.5),[0.05 0.05 0.05]);
%samples = mat2cell(samples,ones(1,n_samples),3);
mu = [0 0 0];
sigma = [0.05 0.05 0.05];
samples = normrnd(repmat(mu,n_samples,1),repmat(sigma,n_samples,1));
samples = mat2cell(samples, ones(1,n_samples), 3);

first = true;

for ind = 1000:length(lidar.ts)-100

    if lidar.ts(ind) - ts0 > 1.0
        disp(['logtime: ' num2str(lidar.ts(ind)) ' realtime: ' num2str(toc())]);
        ts0 = lidar.ts(ind);
    end
    
    %tic    
    
    %% Evaluate costs of each state
    map_int8 = int8(256/(2*cfg.confidence_thresh).*map);
    x_imap = (0:resolution:map_size(1)*resolution) - map_center(1)*resolution;
    y_imap = (0:resolution:map_size(2)*resolution) - map_center(2)*resolution;
    
    costs = zeros(n_samples,1);
    for j = 1:n_samples
        search_range = 0;
        state = samples{j};
       
        x_range = state(1);
        y_range = state(2);

        lidar_hits_search = tform_scan( state, lidar.ranges(:,ind), lidar_norm);
        lidar_corr = [lidar_hits_search(1,:); lidar_hits_search(2,:); zeros(1,num_scans)];

        cost = map_correlation(map_int8, x_imap,y_imap ...
                              ,lidar_corr ...
                              ,x_range,y_range);
        costs(j) = cost;                               
    end
    
    if first
        state = [0 0 0];
        sample_costs = ones(n_samples,1)/n_samples;
        first = false;
    else
        csts = costs - min(costs);
        csts = csts/sum(csts);
        %{
        sample_costs = sample_costs.*csts;
        sample_costs = sample_costs/sum(sample_costs);
        [~,cost_ind] = max(sample_costs);
        %}
        [~,cost_ind] = max(csts);
        state = samples{cost_ind};
    end
    
    %{
    sigma = [0.02 0.02 0.05];
    samples = normrnd(repmat(state,n_samples,1),repmat(sigma,n_samples,1));
    samples = mat2cell(samples, ones(1,n_samples), 3);
    %}
    
    %{
    % If max(sample_costs)/mean(sample_costs) > some threshold, then
    % resample based on distribution 
    % at the moment just resample around max(sample_costs)
    if max(sample_costs)/mean(sample_costs) > 20
        disp('resample');
        %resamples = bsxfun(@times, 2*(rand(n_samples,3)-0.5),[0.1 0.1 0.05]);                
        %samples = bsxfun(@plus, resamples, state);
        mu = state;
        sigma = [0.05 0.05 0.05];
        samples = normrnd(repmat(mu,n_samples,1),repmat(sigma,n_samples,1));
        samples = mat2cell(samples, ones(1,n_samples), 3);
        sample_costs = ones(n_samples,1)/n_samples;
    end
    %}
    
    %{
    figure(2)
    plot( 1:n_samples, sample_costs, '.');
    %}
    
    lidar_hits = tform_scan( state, lidar.ranges(:,ind), lidar_norm);
    set(laserh', 'XData', 1/resolution*lidar_hits(2,:)+map_center(2), 'YData', 1/resolution*lidar_hits(1,:)+map_center(1));

    %state_inds = double(int32(1/resolution*state(1:2) + map_center(1)));
    %state_inds = double(int32(map_size(1)/world_size*state(1:2) + map_center(1)));
    state_inds = double(int32(1/resolution*state(1:2)) + int32(map_center));
        
    % Draw current state
    set(ph, 'XData', state_inds(2), 'YData', state_inds(1) ); 

    % Draw trajectory
    traj = [traj; state_inds];
    set(trajh, 'XData', traj(:,2), 'YData', traj(:,1) );
    
    %lidar_cell_hits = int32(1/resolution*lidar_hits(1:2,:))+map_center(1);
    %lidar_cell_hits = int32(map_size(1)/world_size*lidar_hits(1:2,:))+map_center(1);
    lidar_cell_hits = bsxfun( @plus, int32(1/resolution*lidar_hits(1:2,:)), int32(map_center') );

    
    % Caculate inds where lidar rays passed through.
    [rays_ix,rays_iy] = getMapCellsFromRay(state_inds(1),state_inds(2) ...
                                           , double(lidar_cell_hits(1,:)),double(lidar_cell_hits(2,:)));
    
    %plot original lidar points
    
    %{
    figure(3);
    lidar_cell_hits = bsxfun( @plus, int32(1/resolution*lidar_hits(1:2,:)), int32(map_center') );

  
    %plot map
    imagesc(map_int8);
    hold on;
    plot(lidar_cell_hits(2,:),lidar_cell_hits(1,:),'r.')
        
    input('');
    %}
                    
    %% Apply probabilities to map
   
    hit_inds = sub2ind( map_size, lidar_cell_hits(1,:), lidar_cell_hits(2,:) );
    ray_inds = sub2ind( map_size, rays_ix, rays_iy );

    % Find unique ray_inds that are not in hit_inds
    ray_inds = setdiff( ray_inds, hit_inds );
    
    % Subtract miss probabilities
    map(ray_inds) = max(min(map(ray_inds)-cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
    
    % Add hit probabilities
    % Bound map values by max-min confidence thresholds
    map(hit_inds) = max(min(map(hit_inds)+2.0*cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh);                             
    
    if mod(ind, 1) == 0
        % Plot samples       
        poss = cell2mat(samples);
        pos = bsxfun(@plus, int32(1/resolution*poss(:,1:2)), int32(map_center));
        set(samplesh, 'XData', pos(:,2), 'YData', pos(:,1));
        % Plot best sample
        pstate = int32(1/resolution*state(1:2)) + int32(map_center);
        set(bsampleh, 'XData', pstate(2), 'YData', pstate(1));
        set(im,'CData', scl*(map+cfg.confidence_thresh)) 
        drawnow
    end   
    
    
    %% Sample vel and w to produce a set of states
    [vel, w, dt] = get_vel( ind, dat, cfg );
    
    % For each sample perturb vel and w and update state    
    vels = bsxfun( @plus, normrnd(0,vel_sigma,n_samples,1), vel );
    ws = bsxfun( @plus, normrnd(0,w_sigma,n_samples,1), w );
    samples = arrayfun( @(j) update_motion( samples{j}, vels(j), ws(j), dt ), 1:n_samples, 'UniformOutput', false )';    
    
end
    
    
    
    
    
    

    


