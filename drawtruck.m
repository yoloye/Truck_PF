function [car] = drawtruck(x,y,h,delta,blength,bwidth)
% function drawcar(x,y,h,delta,scale,fig)
% This function plots a car at position x,y heading h with steering angle delta
% and size scale on figure number fig.
% The default x,y = (0,0), h = 0, delta = 0 drives to the right, and scale=1 plots a
% car with a rectangular body of length 1 units and width 0.5 units.

% Define the body shape

body = [-blength/2 -bwidth/2; blength/2 -bwidth/2; blength/2 bwidth/2; -blength/2 bwidth/2; -blength/2 -bwidth/2];

% Define the wheel shape
wwidth= 0.3;
wlength = 0.75;%0.757
wheel = [-wlength/2 -wwidth/2; wlength/2 -wwidth/2; wlength/2 wwidth/2; -wlength/2 wwidth/2; -wlength/2 -wwidth/2];
R = rot2D(delta);
R = R(1:2,1:2);

% rotate the wheel coordinates into the inertial frame using R'
fwheel = (R'*wheel')';

% Define car
n = length(wheel(:,1));
wheel1 = wheel + [body(1,1)*ones(n,1)+1,body(1,2)*ones(n,1)]; %rear right
wheel2 = fwheel + [body(2,1)*ones(n,1)-1,body(2,2)*ones(n,1)]; %front right
wheel3 = fwheel + [body(3,1)*ones(n,1)-1,body(3,2)*ones(n,1)]; %front left
wheel4 = wheel + [body(4,1)*ones(n,1)+1,body(4,2)*ones(n,1)]; %rear left
wheel5 = wheel + [body(1,1)*ones(n,1)+2,body(1,2)*ones(n,1)];
wheel6 = wheel + [body(4,1)*ones(n,1)+2,body(4,2)*ones(n,1)];

car = [body;NaN NaN;
       wheel1;NaN NaN;
       wheel2;NaN NaN;
       wheel5;NaN NaN;
       wheel6;NaN NaN;
       wheel3;NaN NaN;
       wheel4];

% Rotation matrix
R = rot2D(h);
car = (R'*car')';

% Centre
car(:,1) = car(:,1)+x;
car(:,2) = car(:,2)+y;

end