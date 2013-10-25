function new_state = update_motion( state, v, w, dt )
    theta = state(3);
    if w ~= 0
        new_state = state + [-v/w*sin(theta)+v/w*sin(theta+w*dt) ...
                              v/w*cos(theta)-v/w*cos(theta+w*dt) ...
                              w*dt];
    else
        %new_state = state; % this isn't actually correct but for now whatever
        new_state = state + [-v*dt*sin(theta)+v*dt*sin(theta+w*dt) ...
                              v*dt*cos(theta)-v*dt*cos(theta+w*dt) ...
                              w*dt];
    end
end