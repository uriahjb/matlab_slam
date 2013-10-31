%{
    Calculate the likelihood of each particle given measurements
%}
function [particles, measurement_log_likelihood] = measurement_likelihood( ind, particles, lidar, world )       
    % Transform lidar measurements by particle state    
    for pidx = 1:particles.count
        particles.lidar_hits{pidx} = tform_scan( particles.state(pidx,:), lidar.ranges(:,ind), lidar.norm );
    end             
    
    % Create single lidar hits array for all particles
    lidar_hits = cat(2,particles.lidar_hits{:});
    
    % Create mask indicating which lidar values to ignore    
    ignore_msk = repmat(lidar.ranges(:,ind) > lidar.max_range, particles.count, 1);
    
    % Convert to world coords
    lidar_hits_map = 1/world.resolution*lidar_hits;
    lidar_hits_map(1,:) = lidar_hits_map(1,:) + world.center(1);
    lidar_hits_map(2,:) = lidar_hits_map(2,:) + world.center(2);
    %TODO: I can save time here by sampling from a subset of the world map    
    hit_values = interp2( world.map, lidar_hits_map(2,:), lidar_hits_map(1,:), 'linear' );
    
    % Ignore values from non-lidar hits ... not sure how justified this is
    hit_values( ignore_msk ) = 0.0;
    % y = x/(1 + x)  x = logprob, y = prob
    particles.hit_cost = sum(reshape(hit_values, length(lidar.angles), particles.count),1)'/length(lidar.angles);            
    %hit_costs = reshape(hit_values, length(lidar.angles), particles.count)';                
    %prob_hit_costs = exp(hit_costs)./(1+exp(hit_costs));
    
    prob_hit_cost = exp(particles.hit_cost)./(1+exp(particles.hit_cost));        
    %measurement_log_likelihood = sum( log( prob_hit_costs ), 2 );
    measurement_log_likelihood = log(prob_hit_cost);
    particles.normalization = particles.normalization + sum(measurement_log_likelihood);
    
end
