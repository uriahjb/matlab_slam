%{
    A script for testing out my particle filter implmentation

    % TODO: proper weight updating, super essential. map-per-particle
                  
%}


%% Load in data
dat = load_measurements(21);

%% World Configuration

p_confidence_thresh = 0.9999;
%unknown = log((1-p_confidence_thresh)/p_confidence_thresh);
unknown = log(0.5/0.5);

world.resolution = 0.05;
world.width = 60;
world.size = [world.width world.width]./world.resolution;
world.center = world.size./2;
world.map = ones(world.size)*unknown; % initialize map as unknown

%% Robot Configuration
wheel_circ = 2*pi*0.165;
cnt_to_rad = 1/360;

cfg = struct();
cfg.imu_scl = 1050/1023*pi/180;
cfg.imu_bias = 369.75;
cfg.cnt_to_vel = 8.0*wheel_circ*cnt_to_rad;

p_hit = 0.53;
p_miss = 1 - p_hit;
cfg.log_hit = log(p_hit/p_miss);
cfg.confidence_thresh = log(p_confidence_thresh/(1-p_confidence_thresh));
%cfg.unknown = log(0.5/0.5);
cfg.unknown = unknown;

cfg.search_range = world.resolution*4;
cfg.theta_range = 0.1;
cfg.num_thetas = 10;

%% Particle Filter Configuration
particles.count = 100;
particles.variance = [2.0, 1.0];
%particles.variance = [0.5, 0.5];
particles.state = zeros(particles.count,3);
particles.lidar_hits = {};
particles.hit_cost = zeros(particles.count,1);
particles.cost = ones(particles.count,1)/particles.count;
particles.normalization = 0;
particles.Neff = [];


%% For plotting
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

lidar = dat.hokuyo;
lidar_rots = rot(lidar.angles);
num_scans = length(lidar.angles);

ranges_norm = [ones(length(lidar.angles),1) zeros(length(lidar.angles),1)];
lidar.norm = cell2mat(arrayfun( @(j) lidar_rots(:,:,j)*ranges_norm(j,:)', 1:length(lidar.angles), 'UniformOutput', false ));

%% For drawing trajectory, current position and map
figure(1)

range = [0 1.0];
scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);

num_tiles = int16(sqrt(particles.count));

%map_grid = repmat( scl*(world.map+cfg.confidence_thresh), num_tiles, num_tiles );
%im = imshow(map_grid, 'Border', 'tight');
im = imshow(scl*(world.map+cfg.confidence_thresh), 'Border', 'tight');
set(im, 'clipping', 'off');
hold on;
qh = quiver([],[],[],[]);

p_lh = [];
for pidx = 1:particles.count
    lh = line();
    set(lh, 'LineStyle', 'None', 'Marker', '*', 'Color', 'g')
    p_lh = [p_lh lh];
end

min_box_h = line();
set( min_box_h, 'Color', 'r' );

max_box_h = line();
set( max_box_h, 'Color', 'b' );

% Bar graph
figure(2);
phc_bh = bar( particles.hit_cost );

% Neff plot
figure(3);
neff_h = line();

% Particles 
%R = [cos(particles.state(:,3)) sin(particles.state(:,3)); sin(particles.state(:,3)) cos(particles.state(:,3))];
%vy = cos(particles.state(:,3)) + sin(particles.state(:,3));
%qh = quiver(particles.state(2,:), particles.state(1,:), )
figure(1)
%qh = line();
%set( qh, 'LineStyle', 'None', 'Marker', '*', 'Color', 'g');
%qh = quiver([],[],[],[]);

%% Init

start_ind = 800;

state = [0 0 0];
ts0 = lidar.ts(start_ind);


%% Given state and lidar hits update the map
world = update_map( start_ind, state, lidar, world, cfg );    

figure(1);
%map_grid = repmat( scl*(world.map+cfg.confidence_thresh), int16(sqrt(particles.count)), int16(sqrt(particles.count)) );
%set(im,'CData', map_grid)         

set(im,'CData', scl*(world.map+cfg.confidence_thresh))         
drawnow();

%% Plot lidar points for next state

%% Sample noise distributions to create v,w for each particle
% Variance [vel, w]
% Get vel,w
for ind = start_ind:2500
    [vel, w, dt] = get_vel( ind, dat, cfg );

    % Generate vel,w samples for each particle
    %[pgrad] = normrnd(repmat([vel w], particles.count, 1), repmat(particles.variance, particles.count, 1));
    [pgrad] = normrnd(repmat([0.0 0.0], particles.count, 1), repmat(particles.variance, particles.count, 1));
    
    % Calculate probability of each sample
    %{
    sample_probs = normpdf( repmat(particles.variance, particles.count, 1), pgrad );
    log_sample_probs = sum(log(sample_probs./(1-sample_probs)),2);
    particles.cost = particles.cost + log_sample_probs;
    %}
    
    pvel = pgrad(:,1) + vel;
    pw = pgrad(:,2) + w;

    pstates = zeros(particles.count,3);
    plidar_hits = {};
    prev_state = particles.state;
    for pidx = 1:particles.count
        particles.state(pidx, :) = update_motion( particles.state(pidx,:), pvel(pidx), pw(pidx), dt );
        particles.lidar_hits{pidx} = tform_scan( particles.state(pidx,:), lidar.ranges(:,ind), lidar.norm );
    end 

    %% Draw lidar hits for each particle
    [i,j] = ind2sub( [num_tiles, num_tiles], 1:particles.count);
    i = 2*(i-1)*world.center(1);
    j = 2*(j-1)*world.center(2);

    hit_costs = zeros(particles.count, 1);
    
    % Create single lidar hits array for all particles
    lidar_hits = cat(2,particles.lidar_hits{:});
    % Convert to world coords
    lidar_hits_map = 1/world.resolution*lidar_hits;
    lidar_hits_map(1,:) = lidar_hits_map(1,:) + world.center(1);
    lidar_hits_map(2,:) = lidar_hits_map(2,:) + world.center(2);
    %TODO: I can save time here by sampling from a subset of the world map
    hit_values = interp2( world.map, lidar_hits_map(2,:), lidar_hits_map(1,:), 'nearest' );
    % y = x/(1 + x)  x = logprob, y = prob
    particles.hit_cost = sum(reshape(hit_values, 1081, particles.count),1)'/length(lidar.angles);        
    prob_hit_cost = exp(particles.hit_cost)./(1+exp(particles.hit_cost));    
    log_hit_cost = log( prob_hit_cost/sum(prob_hit_cost) );    
    %particles.cost = particles.cost + particles.hit_cost;    
    %particles.cost = prob_hit_cost/sum(prob_hit_cost);
    %particles.cost = particles.cost + log_hit_cost;
    particles.cost = log_hit_cost;
    %particles.normalization = particles.normalization + sum( log_hit_cost );
    
    
    %particles.cost = particles.cost/sum(particles.cost);    
    
    %{
    for pidx = 1:particles.count

        lidar_hits = particles.lidar_hits{pidx};
        lidar_hits_map = 1/world.resolution*lidar_hits;
        lidar_hits_map(1,:) = lidar_hits_map(1,:) + world.center(1);
        lidar_hits_map(2,:) = lidar_hits_map(2,:) + world.center(2);

        %set( p_lh(pidx), 'XData', lidar_hits_map(2,:) + i(pidx), 'YData', lidar_hits_map(1,:) + j(pidx) );

        %% While we are at it calculate the probability cost for each particles hit hypothesis
        hit_values = interp2( world.map, lidar_hits_map(2,:), lidar_hits_map(1,:) );
        % Sum the values without weighting since we assume that p(hit) =
        % p(no_hit)
        particles.hit_cost(pidx) = sum(hit_values)/length(lidar.angles);
        
        particles.cost(pidx) = particles.cost(pidx) + particles.hit_cost(pidx);
    end
    %}

    drawnow();

    %% Calculate Neff
    cost_normalized = exp( particles.cost - min(particles.cost) );
    cost_normalized = cost_normalized/sum(cost_normalized);
    particles.Neff = [particles.Neff 1/sum(cost_normalized.^2)];
    %particles.Neff = [particles.Neff var(particles.cost)];
    
    %{
    figure(3)
    set( neff_h, 'XData', 1:length(particles.Neff), 'YData', particles.Neff );
    drawnow();
    %}
    
    %% Plot particle hit costs
    %{
    figure(2)
    set( phc_bh, 'YData', particles.hit_cost );    
    %}
    %% Draw a boxes around the best and worst samples
    box = 2.0*world.center(1)*[0 1 1 0 0; 0 0 1 1 0];
    [~, mx_idx] = max(particles.cost);
    [~, mn_idx] = min(particles.cost);

    %{
    figure(1)
    set( max_box_h, 'XData', box(1,:)+i(mx_idx), 'YData', box(2,:)+j(mx_idx) );

    set( min_box_h, 'XData', box(1,:)+i(mn_idx), 'YData', box(2,:)+j(mn_idx) );
    drawnow();    
    %}
    
    %input(''); % Pause for user input
    %% Update world with best state update
    world = update_map( ind, particles.state(mx_idx,:), lidar, world, cfg );        
    %map_grid = repmat( scl*(world.map+cfg.confidence_thresh), int16(sqrt(particles.count)), int16(sqrt(particles.count)) );
    %set(im,'CData', map_grid)         
    
    if ~mod(ind, 15)

        set(im,'CData', scl*(world.map+cfg.confidence_thresh))   

        % Draw particles
        %set(qh, 'XData', 1/world.resolution*particles.state(:,2) + world.center(1), 'YData', 1/world.resolution*particles.state(:,1) + world.center(2));
        set(qh, 'XData', 1/world.resolution*particles.state(:,2) + world.center(1), 'YData', 1/world.resolution*particles.state(:,1) + world.center(2) ...
              , 'UData', 20/world.resolution*(particles.state(:,2)-prev_state(:,2)), 'VData', 20/world.resolution*(particles.state(:,1)-prev_state(:,1)) );
        drawnow();
    end
    
    
    %% Resample particles if its required
    
    if particles.Neff(end) < particles.count/2
        
    % TODO: evaluate that this is correct
    %if ~mod(ind, 100)
        cost_normalized = exp( particles.cost - min(particles.cost) );
        cost_normalized = cost_normalized/sum(cost_normalized);
        [sorted_pc, inds_pc] = sort(cost_normalized);
        parents_nrmd = sorted_pc/sum(sorted_pc);
        
        % Systematic Resampling
        weights = cumsum(parents_nrmd);
        % Compute normalizing factor
        step = 1/particles.count;
        % Compute random starting point
        start = step*rand(1);
        % Compute selection points
        selection_points = start:step:(state+particles.count*step);
        j = 1;
        idx = [];
        for i = 1:particles.count
            while selection_points(i) > weights(j)
                j = j+1;
            end
            idx = [idx j];
        end
        parent_idx = inds_pc( idx );
        
        % Copy particle values over
        particles.state = particles.state(parent_idx,:);
        particles.hit_cost = particles.hit_cost(parent_idx);
        particles.cost = particles.cost(parent_idx);
        particles.lidar_hits = particles.lidar_hits(parent_idx);
        
        
        %{
        Very Random resampling
        %parents_nrmd = sorted_pc;        
        idx = sum(bsxfun(@ge, rand(1,particles.count), cumsum(parents_nrmd) ))' + 1;
        parent_idx = inds_pc( idx );
        % Copy particle values over
        particles.state = particles.state(parent_idx,:);
        particles.hit_cost = particles.hit_cost(parent_idx);
        particles.cost = particles.cost(parent_idx);
        particles.lidar_hits = particles.lidar_hits(parent_idx);
        %}
        %{
        OLD BROKEN ALGORITHM
        resample_percent = 0.7;
        [sorted_pc, inds_pc] = sort(particles.cost);
        num2resample = int16(particles.count*resample_percent);    
        parents = sorted_pc((end-num2resample+1):end);
        parents_nrmd = exp(parents)/sum(exp(parents));
        % Get the indices of the new state of each particle. 
        % RESAMPLING STILL HAS ERRORS, WRONG SIZES FOR NUM2RESAMPL AND
        % PARENTS_NRMD
        idx = sum(bsxfun(@ge, rand(1,num2resample), cumsum(parents_nrmd) ))';
        parent_idx = inds_pc(int16(idx+1) + particles.count-num2resample);
        % Update particle values 
        children_idx = inds_pc(1:num2resample);
        particles.state(children_idx,:) = particles.state(parent_idx,:);
        particles.hit_cost(children_idx) = particles.hit_cost(parent_idx);
        particles.cost(children_idx) = particles.cost(parent_idx);
        particles.lidar_hits(children_idx) = particles.lidar_hits(parent_idx);
        disp(['Resample: ' num2str(ind)]);
        %}
    end
    
end
