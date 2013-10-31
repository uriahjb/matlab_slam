%{
    Use systematic resampling algorithm to resample particles based on 
    their weights

    This isn't particularly optimized or anything but seems to work well
    enough. 
    
    Note: there a bunch of resampling algorithms out their and probably 
          worth exploring 
%}

function particles = systematic_resample( particles )
    %cost_normalized = exp( particles.cost - max(particles.cost) );
    cost_normalized = exp( particles.cost );
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
    selection_points = start:step:(start+particles.count*step);
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
    %particles.cost = particles.cost(parent_idx);
    particles.cost = log(ones(particles.count,1)/particles.count);
    particles.normalization = sum( particles.cost );
    
    particles.lidar_hits = particles.lidar_hits(parent_idx);
        
    %particles.world = particles.world{parent_idx};
    %particles.world(:) = particles.world(parent_idx)
end