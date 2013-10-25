dt = 0.1;

state = [0 0 pi/4];
v = 1;
%w = 2*pi*dt;
w = 0;

close all;
figure(1);
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

for i = 1:50
    state = update_motion( state, v, w, dt );
    v_tf = tforms(state)*vec;
    plot( v_tf(1,1:2), v_tf(2,1:2), '-r');
    hold on;
    plot( v_tf(1,3:4), v_tf(2,3:4), '-b');
    drawnow;
    axis equal;
    %input('');
end

