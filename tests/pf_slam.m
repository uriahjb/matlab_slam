%{
    Implementation Dynamic Map Monte-Carlo Localization
%}

%% Load in data
dat = load_measurements(20);

%% World Configuration

p_confidence_thresh = 0.99999;
unknown = log(0.5/0.5);

world.resolution = 0.2;
world.width = 70;
world.size = [world.width world.width]./world.resolution;
world.center = world.size./2;
world.map = ones(world.size)*unknown; % initialize map as unknown

%% Robot Configuration
wheel_circ = 2*pi*0.165;
cnt_to_rad = 1/360;

cfg = struct();
cfg.imu_scl = 1050/1023*pi/180;
cfg.imu_bias = 370.0;
cfg.cnt_to_vel = 8.0*wheel_circ*cnt_to_rad;

p_hit = 0.52;
p_miss = 1 - p_hit;
cfg.log_hit = log(p_hit/p_miss);
cfg.confidence_thresh = log(p_confidence_thresh/(1-p_confidence_thresh));
cfg.unknown = unknown;

cfg.search_range = world.resolution*4;
cfg.theta_range = 0.1;
cfg.num_thetas = 10;

%% Particle Filter Configuration
particles.count = 100;
particles.variance = [0.5, 0.1];
%particles.variance = [1.0, 0.2];
particles.state = zeros(particles.count,3);
particles.lidar_hits = {};
particles.hit_cost = zeros(particles.count,1);
particles.cost = log(ones(particles.count,1)/particles.count);
particles.world = {};
particles.normalization = sum(particles.cost);
particles.Neff = [];

% Initialize world for each particle
for px = 1:particles.count
    particles.world{px} = world;
end


%% Lidar Configuration
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

lidar = dat.hokuyo;
lidar_rots = rot(lidar.angles);
num_scans = length(lidar.angles);

ranges_norm = [ones(length(lidar.angles),1) zeros(length(lidar.angles),1)];
lidar.norm = cell2mat(arrayfun( @(j) lidar_rots(:,:,j)*ranges_norm(j,:)', 1:length(lidar.angles), 'UniformOutput', false ));
lidar.max_range = 26.0;

%% For drawing trajectory, current position and map
figure(1)

range = [0 1.0];
scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);

num_tiles = int16(sqrt(particles.count));

map_grid = repmat( scl*(world.map+cfg.confidence_thresh), num_tiles, num_tiles );
im = imshow(map_grid, 'Border', 'tight');
%im = imshow(scl*(world.map+cfg.confidence_thresh), 'Border', 'tight');
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


%% Init

start_ind = 250;

state = [0 0 0];
ts0 = lidar.ts(start_ind);

for ind = start_ind:4000
    % Get sensor measurements
    [vel, w, dt] = get_vel( ind, dat, cfg );
    
    % Previous state used in plotting
    prev_state = particles.state;
    
    % Sample motion model distribution for each particle
    [particles, motion_log_likelihood] = sample_particle_motion( ind, particles, vel, w, dt );
        
    % Update particle weights given motion log-likelihoods
    %particles.cost = particles.cost + motion_log_likelihood;    
    
    % Calculate likelihood of particle given measurements
    [particles, measurement_log_likelihood] = measurement_likelihood_mm( ind, particles, lidar, world );        
    % Update particle weights with measurement likelihoods
    particles.cost = particles.cost + measurement_log_likelihood;
    
    % Update map hypothesis for each particle    
    for px = 1:particles.count
        particles.world{px} = update_map( ind, particles.state(px,:), lidar, particles.world{px}, cfg );        
    end

    % Set world to draw to that of best particle
    [~, mx_idx] = max(particles.cost);
    world = particles.world{mx_idx};
    
    % Calculate Neff
    Neff = calculate_neff( particles )
    
    % Resample particles if necessary 
    if Neff < particles.count/4
        particles = systematic_resample( particles );
    end

    % Draw world and particles
    
    if ~mod(ind, 30)
        % Update map_grid with each particles map
        [~, psrt] = sort( particles.cost, 'descend' );
        [i,j] = ind2sub( [num_tiles, num_tiles], 1:particles.count );
        for pidx = 1:particles.count            
            map_grid( i(pidx)*world.size(1)-world.size(1)+1:world.size(1)*i(pidx) ...
                    , j(pidx)*world.size(1)-world.size(1)+1:world.size(1)*j(pidx) ) ...
                    = scl*(particles.world{psrt(pidx)}.map+cfg.confidence_thresh);
        end
        set( im, 'CData', map_grid );
        
        %set(im,'CData', scl*(world.map+cfg.confidence_thresh))   

        % Draw particles        
        set(qh, 'XData', 1/world.resolution*particles.state(:,2) + world.center(1), 'YData', 1/world.resolution*particles.state(:,1) + world.center(2) ...
              , 'UData', 20/world.resolution*(particles.state(:,2)-prev_state(:,2)), 'VData', 20/world.resolution*(particles.state(:,1)-prev_state(:,1)) );        
          
         % Draw min and max boxes
        box = 2.0*world.center(1)*[0 1 1 0 0; 0 0 1 1 0];
        [~, mx_idx] = max(particles.cost);
        [~, mn_idx] = min(particles.cost);

        set( max_box_h, 'XData', box(1,:)+2*(i(mx_idx)-1)*world.center(1), 'YData', box(2,:)+2*(j(mx_idx)-1)*world.center(2) );
        set( min_box_h, 'XData', box(1,:)+2*(i(mn_idx)-1)*world.center(1), 'YData', box(2,:)+2*(j(mn_idx)-1)*world.center(2) );

    end 
    
   
    drawnow();
    
end

