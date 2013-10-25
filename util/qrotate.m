function v_new = qrotate(q,v)
    %{
    v_sz = size(v);
    vcat = cat(1, zeros(1,v_sz(2)), v);
    %}
    v_new = qmult(qmult(q,[0; v]), qconj(q));
end