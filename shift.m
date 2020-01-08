function [t0, x0, u0, x_x_1] = shift(T, t0, x0, u, f, x_x_1, V_1)    
    x_x_1 = x_x_1 +V_1*T;   
    st = x0;
    con = u(1,:)';
    f_value = f(st,con);
    st = st + (T * f_value);
    x0 = full(st);
    t0 = t0 + T;
    u0 = [u(2:size(u,1),:); u(size(u,1), :)];
    % Update u0, u from this step is the u0 of next step
    % This has to be change if there are more than one input
end