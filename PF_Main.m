% ====================================================================================================================
%                                                      Semi-Truck's
%                                                      State Space
% ====================================================================================================================
addpath('C:\Program Files\casadi')
import casadi.*

% =========================================================================
%                             Simulition setup
% =========================================================================
T = 5; % Total simulate time
delta_T = 0.05; % Sample time(s)
N = 100; % Prediction Horizon	
t_vector = (0:delta_T:T);

% =========================================================================
%                           System Parameters setup
% =========================================================================
%Demension of semi truck
bwidth_truck =2.5; %width of truck
bwidth_trailer = 2.5; %width of tralior
bwidth = 2.5;
l_fa = 1; %Distance from front axle to front end of the tractor
l_fb = 4; %Wheelbase if truck
l_fc = 1.5;%Distance from rear axle to rear end of the tractor
l_ra = 1.5; %Distance from hinge point to front end of the trailer
l_rb = 6.5; %Distance from hinge point to rear axle
l_rc = 2; %Distance from rear axle to rear end of the trailer
blength_truck = l_fa+l_fb+l_fc; %body length of truck
blength_trailer = l_ra+l_rb+l_rc; %body length of tralior

% =========================================================================
%                             States setup
% =========================================================================
theta_f = SX.sym('theta_f'); %Heading/Yaw angle of the tractor; 
theta_r = SX.sym('theta_r');%Heading/Yaw angle of the trailer;
gamma = SX.sym('gamma');  %The angle between the tractor and the trailer; 
x_f = SX.sym('x_f'); y_f = SX.sym('y_f');% x_f The midpoint of the equivalent rear axle of the tractor
                                         %      The articulation point of the tractor and the trailer
x_ff = SX.sym('x_f'); y_ff = SX.sym('y_f'); % The midpoint of the front end of the tractor; 
x_fr = SX.sym('x_fr'); y_fr = SX.sym('y_fr');% The midpoint of the rear end of the tractor; 
x_rf = SX.sym('x_rf'); y_rf = SX.sym('y_rf');% The midpoint of the front end of the trailer;
x_rr = SX.sym('x_rr'); y_rr = SX.sym('y_rr');% The midpoint of the rear end of the trailer; 

states = [theta_f; theta_r; gamma; x_f; y_f; x_ff; y_ff; x_fr; y_fr; x_rf; y_rf; x_rr; y_rr];
n_states = length(states);

% =========================================================================
%                        Control Inputs setup
% =========================================================================
delta = SX.sym('delta'); %The angle of the steering wheels of the semi-trailer
v_f = SX.sym('v_f'); %Longitudial velocity of tractor

controls = [delta; v_f];
n_controls = length(controls);

% =========================================================================
%                        State Space setup
% =========================================================================
theta_f_dot = (v_f*tan(delta))/l_fb; 
theta_r_dot = (v_f*sin(gamma))/l_rb;
gamma_dot = (v_f*tan(delta))/l_fb - (v_f*sin(gamma))/l_rb; 
x_f_dot = v_f*cos(theta_f); 
y_f_dot = v_f*sin(theta_f); 
x_ff_dot = v_f*cos(theta_f)-(l_fa+l_fb)*theta_f_dot*sin(theta_f); 
y_ff_dot = v_f*sin(theta_f)+(l_fa+l_fb)*theta_f_dot*cos(theta_f);  
x_fr_dot = v_f*cos(theta_f)+l_fc*theta_f_dot*sin(theta_f); 
y_fr_dot = v_f*sin(theta_f)-l_fc*theta_f_dot*cos(theta_f);  
x_rf_dot = v_f*cos(gamma)*cos(theta_r)-(l_ra+l_rb)*theta_r_dot*sin(theta_r); 
y_rf_dot = v_f*cos(gamma)*sin(theta_r)+(l_ra+l_rb)*theta_r_dot*cos(theta_r); 
x_rr_dot = v_f*cos(gamma)*cos(theta_r)+l_rc*theta_r_dot*sin(theta_r); 
y_rr_dot = v_f*cos(gamma)*sin(theta_r)-l_rc*theta_r_dot*cos(theta_r); 

rhs = [theta_f_dot; theta_r_dot; gamma_dot; x_f_dot; y_f_dot; ...
    x_ff_dot; y_ff_dot; x_fr_dot; y_fr_dot; x_rf_dot; y_rf_dot; x_rr_dot; y_rr_dot];

% =========================================================================
%                        Initial State setup
% =========================================================================
% states = [theta_f; theta_r; gamma; x_f; y_f; x_ff; y_ff; x_fr; y_fr; x_rf; y_rf; x_rr; y_rr];
x_f_0 = 5;
y_f_0 = 1.7;
[x_ff_0, y_ff_0, x_fr_0, y_fr_0, x_rf_0, y_rf_0, x_rr_0, y_rr_0] = position_calculation(0, 0, x_f_0,y_f_0,l_fa, l_fb, l_fc, l_ra, l_rb, l_rc);

% x_ff_0 = x_f_0 + l_fb;
% x_fr_0 = x_f_0 - l_fc;
% x_rf_0 = x_f_0 + l_ra;
% x_rr_0 = x_f_0 -(l_rb+l_rc);
x0 = [0;0;0;x_f_0 ;y_f_0 ;x_ff_0;y_ff_0;x_fr_0;y_fr_0;x_rf_0;y_rf_0;x_rr_0;y_rr_0;];% initialize states, based on number of states
% xs = [0;2e-5;2e-5;-100;2e-5;2e-5;2e-5;2e-5;2e-5;2e-5;2e-5;2e-5;2e-5;]; %Reference state

num_obstacles = 1;
x_x_1 = 20; %initial longitudial position of first obstacle;
x_y_1 = 1.7; %initial lateral position of first obstacle;
V_1 = 5; %Velocity of first obstacle;

% ====================================================================================================================
%                                   Transfer Optimal control problam to NLP
% ====================================================================================================================
f = Function('f', {states, controls}, {rhs});
% Define a function "f", it will take states and controls as variable and return rhs

U = SX.sym('U', n_controls, N); %Store information of Decision Variables(controls)

p = SX.sym('p', n_states + N*(n_states) + num_obstacles ); % Optimization problem parameteres
% 1st n_states: Current State
% 2nd n_states: Reference State along whole prediction horizon
% n_controls For constraint on Control related to previous state 
% NO REFERENCE CONTROL HERE!!!
% Parameters 'p', this part of this matrix(Length(n_states)) store 
% information of initial state of x(x0), and second part of it
% contains information of the reference state.

X = SX.sym('x', n_states,(N+1));
% A vector that contains states over the pridiction horizon period
% and the N+1 means that it contains the intial condition from last
% control period.

obj = 0 ; %objective function, will be continous update
g = [];   %constrain vector

% ====================================================================================================================
%                                                  Weighting matrix                    
% ====================================================================================================================
% Weighting matrix of states
Q = zeros(n_states, n_states); 
Q(1,1) =1; Q(2,2) = 0; Q(3,3) = 0; 
Q(4,4) =10; Q(5,5) = 10; Q(6,6) = 0;
Q(7,7) =0; Q(8,8) =0; Q(9,9) = 0; 
Q(10,10) = 0;Q(11,11) =0; Q(12,12) = 0; 
Q(13,13) = 0;

% % Weighting matrix of states
% Q = zeros(n_states, n_states); 
% Q(1,1) =0; Q(2,2) = 0; Q(3,3) = 0; 
% Q(4,4) =10; Q(5,5) = 10; Q(6,6) = 10;
% Q(7,7) =10; Q(8,8) =10; Q(9,9) =10; 
% Q(10,10) = 10;Q(11,11) =10; Q(12,12) = 10; 
% Q(13,13) = 10;

% %Weighting matrix of states
% Q = zeros(n_states, n_states); 
% Q(1,1) =0; Q(2,2) = 0; Q(3,3) = 0; 
% Q(4,4) =50; Q(5,5) = 50; Q(6,6) = 10;
% Q(7,7) =10; Q(8,8) =10; Q(9,9) = 10; 
% Q(10,10) = 10;Q(11,11) =10; Q(12,12) = 10; 
% Q(13,13) = 10;

%Weighting matrix of controls
R = zeros(n_controls, n_controls); 
R(1,1) = 10; R(2,2) = 1;

% ====================================================================================================================
%                                                  Set up NLP solver                    
% ====================================================================================================================
st = X(:, 1); 
% steer = U(2,1) ;
g = [g; st - p(1:n_states)];% initial condition constraints
% The following loop is aim to calculate the cost function and constrains
x_prev = x_f_0;
y_prev = y_f_0;
for i = 1:N
        obstacle_position = p((n_states *(N+1))+1) + i*V_1;
        st = X(:, i);
        con = U(:, i);
        v_x = (X(4, i)-x_prev)/delta_T;
        v_y = (X(5, i)-y_prev)/delta_T;
        zz =   10*NonCrossable_PF(X(4, i), X(5, i),obstacle_position, x_y_1, v_x, v_y, V_1, 2, 1);
        obj = obj +(st - p((n_states *i+1):(n_states * (i+1))))' * Q *...
            (st - p((n_states *i+1):(n_states * (i+1)))) + con' * R * con+zz;
        x_prev = X(4, i);
        y_prev = X(5, i);
        %p[state_initial,state_reference(i),state_reference(i+1)...state_reference(N)]
        st_next = X(:, i + 1);
        f_value = f(st, con); %this is define in previous step, it will return rhs
        st_next_predict = st + (delta_T*f_value);
        % the state of next time step
        g = [g; st_next - st_next_predict];
        %paraller compute the constrains, it equals
        %state_next - state_next_predict
end

% Form the NLP 
OPT_variables = [reshape(X, n_states * (N+1),1); reshape(U, n_controls * N,1)];
% reshape all predict states and controls variable to a one column vector
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', p);

opts = struct;
opts.ipopt.max_iter = 3000;
% the algorithm will terminate with an error after max_iter times iterations
opts.ipopt.print_level = 0;
% The larger the value, the more detail in output. [0 12]
opts.print_time = 0;
% A boolean value. print informatin of execution time
opts.ipopt.acceptable_tol = 1e-15;
% The convergence tolerance
opts.ipopt.acceptable_obj_change_tol = 1e-3;
%Stop criterion
solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

%Following constraint should be adjust based on specific problem
args = struct;

args.lbg(1: n_states * (N+1)) = 0; % -1e-20  % Equality constraints
args.ubg(1: n_states * (N+1)) = 0; % 1e-20  % Equality constraints

%Heading of the tractor;
args.lbx(1: n_states: n_states * (N+1), 1) = -inf;
args.ubx(1: n_states: n_states * (N+1), 1) = inf;

%Heading of the trailer;
args.lbx(2: n_states: n_states * (N+1), 1) = -inf;
args.ubx(2: n_states: n_states * (N+1), 1) = inf;

%The angle between the tractor and the trailer; 
args.lbx(3: n_states: n_states * (N+1), 1) = -inf;
args.ubx(3: n_states: n_states * (N+1), 1) = inf; % consider width of vehicle body

%X_hinge point
args.lbx(4: n_states: n_states * (N+1), 1) = -inf;
args.ubx(4: n_states: n_states * (N+1), 1) = inf;

%Y_hinge point
args.lbx(5: n_states: n_states * (N+1), 1) = -inf;
args.ubx(5: n_states: n_states * (N+1), 1) = inf ;

% x_ff The midpoint of the front end of the tractor; 
args.lbx(6: n_states: n_states * (N+1), 1) = -inf;
args.ubx(6: n_states: n_states * (N+1), 1) = inf;

%  y_ff The midpoint of the front end of the tractor; 
args.lbx(7: n_states: n_states * (N+1), 1) = -inf;
args.ubx(7: n_states: n_states * (N+1), 1) = inf;

% x_fr The midpoint of the rear end of the tractor;
args.lbx(8: n_states: n_states * (N+1), 1) = -inf;
args.ubx(8: n_states: n_states * (N+1), 1) = inf;

% y_fr The midpoint of the rear end of the tractor;
args.lbx(9: n_states: n_states * (N+1), 1) = -inf;
args.ubx(9: n_states: n_states * (N+1), 1) = inf;

% x_rf The midpoint of the front end of the trailer; 
args.lbx(10: n_states: n_states * (N+1), 1) = -inf;
args.ubx(10: n_states: n_states * (N+1), 1) = inf;

% y_rf The midpoint of the front end of the trailer; 
args.lbx(11: n_states: n_states * (N+1), 1) = -inf;
args.ubx(11: n_states: n_states * (N+1), 1) = inf;

% x_rr The midpoint of the rear end of the trailer; 
args.lbx(12: n_states: n_states * (N+1), 1) = -inf;
args.ubx(12: n_states: n_states * (N+1), 1) = inf;

% y_rr The midpoint of the rear end of the trailer; 
args.lbx(13: n_states: n_states * (N+1), 1) = -inf;
args.ubx(13: n_states: n_states * (N+1), 1) = inf;

%Constrains of control inupt
%Steering angle
args.lbx(n_states*(N+1)+1: 1: n_states * (N+1) +n_controls * N,1) = -pi/3;
args.ubx(n_states*(N+1)+1: 1: n_states * (N+1) +n_controls * N,1) = pi/3;

%Velocity
args.lbx(n_states*(N+1)+2: 2: n_states * (N+1) +n_controls * N,1) = -5;
args.ubx(n_states*(N+1)+2: 2: n_states * (N+1) +n_controls * N,1) = 27;

% ====================================================================================================================
%                                              Set up the simulation loop
% ====================================================================================================================
t0 = 0;

xx(:, 1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N, n_controls); % two control inputs for each robot
X0 = repmat(x0, 1, N+1)'; % initialization of the states decision variables
% refer as repeat matrix, a n+1 * 1 cell matrix , and each cell contains
% the matrix x0, the cell to mat

sim_time = T;
mpciter = 0;
xx1 = [];
u_cl = [];
x_p_1 = [];
reference_matrix = []; %Record reference for plot
circle_time = 30;
x = 10 ; y= 0;
main_loop = tic; % timer on
% x = 500 ; y= 0;
x_f_ref_prev = 50;
y_f_ref_prev = 50;
x_ref_matrix = [];
while(mpciter < sim_time/delta_T)
%   while mpciter < sim_time/T)
%   this condition is check the error
%   args.p = [x0 ; xs;u_prev(1,1); u_prev(1,2); x_x_1];
    current_time = mpciter*delta_T;  %new - get the current time
%     args.p = [x0 ; xs;u_prev(1,1); u_prev(1,2)];
    args.p(1:13) = x0;
    
    for i = 1:N %new - set the reference to track
        t_predict = current_time + (i-1)*delta_T; % predicted time instant
        x_f_ref = 15*t_predict;
        y_f_ref = 1.7;
               
        x_f_ref_prev = x_f_ref;
        y_f_ref_prev = y_f_ref;     
        reference_matrix(mpciter+1,i,1) = x_f_ref;
        reference_matrix(mpciter+1,i,2) = y_f_ref;
%         (n_states *i):(n_states * (i+1))

%         x_ff_ref = x_f_ref + l_fb;
%         x_fr_ref = x_f_ref - l_fc;
%         x_rf_ref = x_f_ref + l_ra;
%         x_rr_ref = x_f_ref -(l_rb+l_rc);
        [x_ff_ref, y_ff_ref, x_fr_ref, y_fr_ref, x_rf_ref, y_rf_ref, x_rr_ref, y_rr_ref] = position_calculation(theta_f_ref, theta_f_ref, x_f_ref,y_f_ref,l_fa, l_fb, l_fc, l_ra, l_rb, l_rc);
        x_ref_matrix = [x_ref_matrix ; theta_f_ref];
        args.p((n_states *i+1):(n_states *(i+1))) = [theta_f_ref,theta_f_ref,0,x_f_ref, y_f_ref,x_ff_ref,y_f_ref,x_fr_ref,y_f_ref,x_rf_ref,y_f_ref,x_rr_ref,y_f_ref];
        args.p((n_states *(N+1))+1) = x_x_1;
%         args.p((n_states *i+1):(n_states *(i+1))) = [0,0,0,x_f_ref, y_f_ref,0,0,0,0,0,0,0,0];
%do I need reference for all state?
    end
  
    args.x0 = [reshape(X0',n_states * (N+1),1); reshape(u0', n_controls * N,1)];
    sol = solver('x0',args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
    u = reshape(full(sol.x(n_states * (N+1)+1:end))', n_controls , N)'; % Get solution
    xx1(:,1:n_states, mpciter+1) = reshape(full(sol.x(1:n_states * (N+1)))', n_states, N+1)';
%     this is a 3-D matrix, get controls from solution
%     if (u(1,2) - u_cl(i,2)) > (0.005*T)
%         u(1,2) = u_cl(i,2) + (0.005*T);
%     elseif (u(1,2) - u_cl(i,2)) < -(0.01*T)
%         u(1,2) = u_cl(i,2) - (0.01*T);
%     end
    u_cl = [u_cl; u(1,:)];
%     [m,n] = size(u_cl);
%     u_prev(1,1) =u_cl(m,1);
%     u_prev(1,2) =u_cl(m,2);
%   This matrix save the information of control strategy
%   This is the most important information, use this to plot
    t(mpciter+1) = t0;
    x_p_1 = [x_p_1 ; x_x_1];
    [t0, x0, u0, x_x_1] = shift(delta_T, t0, x0, u, f, x_x_1,V_1);
    xx(:, mpciter + 2) = x0;
%   This matrix save the information of state
%   This is the most important information, use this to plot
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % get solution TRAJECTORY
    X0 = [X0(2:end,:); X0(end,:)];
    mpciter = mpciter + 1                   
end
main_loop_time = toc(main_loop);
average_mpc_time = main_loop_time/(mpciter+1);

 Draw_MPC_Truck_Tracking(T, delta_T, N, xx, u_cl, reference_matrix, blength_truck, blength_trailer,bwidth);

figure(2)
subplot(2,1,1);
plot(u_cl(:,1));
subplot(2,1,2);
plot(u_cl(:,2));

