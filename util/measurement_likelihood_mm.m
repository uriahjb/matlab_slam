%{
    Measurement likelihood of each particle given its map
%}

function [particles, measurement_log_likelihood] = measurement_likelihood_mm( ind, particles, lidar, world )       

    % Values to ignore
    ignore_msk = lidar.ranges(:,ind) > lidar.max_range;

    % Transform lidar measurements by particle state    
    for pidx = 1:particles.count
        particles.lidar_hits{pidx} = tform_scan( particles.state(pidx,:), lidar.ranges(:,ind), lidar.norm );
        
        % Create mask indicating which lidar values to ignore    
        lidar_hits = particles.lidar_hits{pidx};
        
        % Convert to world coords
        lidar_hits_map = 1/world.resolution*lidar_hits;
        lidar_hits_map(1,:) = lidar_hits_map(1,:) + world.center(1);
        lidar_hits_map(2,:) = lidar_hits_map(2,:) + world.center(2);
        %TODO: I can save time here by sampling from a subset of the world map    
        inds = sub2ind( particles.world{pidx}.size, round(lidar_hits_map(1,:)), round(lidar_hits_map(2,:)) );
        hit_values = particles.world{pidx}.map(inds);
        %hit_values = interp2( particles.world{pidx}.map, lidar_hits_map(2,:), lidar_hits_map(1,:), 'nearest' );

        % Ignore values from non-lidar hits ... not sure how justified this is
        hit_values( ignore_msk ) = 0.0;        
        particles.hit_cost(pidx) = sum( hit_values )/length(lidar.angles);        
        
    end             
           
    prob_hit_cost = exp(particles.hit_cost)./(1+exp(particles.hit_cost));        
    %measurement_log_likelihood = log( prob_hit_cost/sum(prob_hit_cost) );   
    measurement_log_likelihood = log( prob_hit_cost );
    particles.normalization = particles.normalization + sum(measurement_log_likelihood);
     
    
end
