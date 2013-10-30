%{
    Update particle states by sampling the given motion model and return
    the log likelihoods provided by the motion model. 
    
    Here the velocity and angular velocity are specified in terms of a 
    normal distribution
%}

function [particles, motion_log_likelihood] = sample_particle_motion( ind, particles, vel, w, dt )
    
    % Generate vel,w samples for each particle    
    [pgrad] = normrnd(repmat([0.0 0.0], particles.count, 1), repmat(particles.variance, particles.count, 1));
    
    % Calculate probability of each sample    
    sample_probs = normpdf( repmat(particles.variance, particles.count, 1), pgrad );
    motion_log_likelihood = sum(log(sample_probs),2);
    particles.normalization = particles.normalization + sum(motion_log_likelihood);
    
    pvel = pgrad(:,1) + vel;
    pw = pgrad(:,2) + w;

    % Update particle states    
    for pidx = 1:particles.count
        particles.state(pidx, :) = update_motion( particles.state(pidx,:), pvel(pidx), pw(pidx), dt );
    end 
    
end