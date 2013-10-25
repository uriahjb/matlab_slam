%{
    Given a test file post-process to produce a more beautiful higher
    resolution map
%}

%% Initialize kinect parameters

kinect.rgb_ts = rgb_ts;
kinect.depth_ts = depth_ts;
kinect.zdepth = zdepth;
kinect.jrgb = jrgb;

%% World Configuration
world.resolution = 0.025;
world.width = 60;
world.size = [world.width world.width]./world.resolution;
world.center = world.size./2;
world.map = ones(world.size)*log(0.5/0.5); % initialize map as unknown
world.map_rgb = zeros(world.size(1), world.size(2), 3);

%% Mapping results
cfg = res.cfg;
traj = res.traj;
lidar = res.lidar;

%% For drawing trajectory, current position and map
figure(1)

%{
range = [0 1.0];
scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);
im = imshow(scl*(world.map+cfg.confidence_thresh), 'Border', 'tight');
%}
im = imshow( world.map_rgb );

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

trajplot = [];

%% Init
filxy = ones(3,1)/3;
trajfil = [conv(double(traj(:,1)), filxy, 'valid') conv(double(traj(:,2)), filxy, 'valid')];

st_i = int32((length(lidar.ts) - size(trajfil,1) - 100)/2);

for ind = 1:length(lidar.ts)-100
    %% Given state and lidar hits update the map
    disp(ind);
    state = [trajfil(ind,:) traj(ind,3)];
    world = update_map( ind, state, lidar, world, cfg );    
    
    %% Given state and kinect data project ground onto map
    if mod(ind, 10) == 0 
        world = update_ground( ind, state, kinect, lidar, world, res.t0 );        
        set(im, 'CData', world.map_rgb./255 );
        drawnow
    end
   
    %% Draw trajectory and map
    
    state_inds = 1/world.resolution*state(1:2) + world.center;
        
    % Draw current state
    set(ph, 'XData', state_inds(2), 'YData', state_inds(1) ); 

    % Draw trajectory
    trajplot = [trajplot; state_inds];   
    set(trajh, 'XData', trajplot(:,2), 'YData', trajplot(:,1) );

    %{
    if mod(ind, 25) == 0
        set(im,'CData', scl*(world.map+cfg.confidence_thresh))         
        drawnow
    end 
    %}    
end


    