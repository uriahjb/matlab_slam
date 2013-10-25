function q_delta = qdelta( w, delta_t )
    w_norm = norm(w);
    alpha = w_norm*delta_t;
    if w_norm < 1e-16
        e = [0 0 0]';
    else
        e = w/w_norm;
    end
    q_delta = [cos(alpha/2); e.*sin(alpha/2)];
end