% Another one grabbed from peter corke
function euler = r2eul( m )
    eps = 0.001;
    
    euler = zeros(1,3);
    
	% Method as per Paul, p 69.
	% phi = atan2(ay, ax)
	% Only positive phi is returned.
	if abs(m(1,3)) < eps && abs(m(2,3)) < eps
		% singularity
		euler(1) = 0;
		sp = 0;
		cp = 1;
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
	else
		euler(1) = atan2(m(2,3), m(1,3));
		sp = sin(euler(1));
		cp = cos(euler(1));
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
    end
end