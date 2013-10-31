function Neff = calculate_neff( particles )
    %v = particles.cost - particles.normalization;
    %cost_normalized = exp( particles.cost - min(particles.cost) );
    %cost_normalized = exp( v - max(v) );
    cost_normalized = exp( particles.cost );
    cost_normalized = cost_normalized/sum(cost_normalized);
    Neff = 1/sum(cost_normalized.^2);
    particles.Neff = [particles.Neff Neff];    
end
