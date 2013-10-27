function Neff = calculate_neff( particles )
    cost_normalized = exp( particles.cost - min(particles.cost) );
    cost_normalized = cost_normalized/sum(cost_normalized);
    Neff = 1/sum(cost_normalized.^2);
    particles.Neff = [particles.Neff Neff];    
end
