function q_inv = qinv(q)
    q_inv = qconj(q)/qnorm(q); % normalizing isn't necessary for a unit quaternion but whatever
end