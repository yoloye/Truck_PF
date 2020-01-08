function Draw_MPC_Truck_Tracking(T, delta_T, N, xx, u_cl, reference_matrix,blength_truck, blength_trailer,bwidth)
state_matrix = xx;
figure(1)
actual_x = [];
actual_y = [];
for i= 1:1:(T/delta_T)
x_f = state_matrix(4,i);
y_f = state_matrix(5,i);
x_ff = state_matrix(6,i);
y_ff = state_matrix(7,i);
x_fr = state_matrix(8,i);
y_fr = state_matrix(9,i);
x_rf = state_matrix(10,i);
y_rf = state_matrix(11,i);
x_rr = state_matrix(12,i);
y_rr = state_matrix(13,i);
truck_yaw = state_matrix(1,i);
trailer_yaw = state_matrix(2,i); 

x_truck = (x_ff + x_fr)/2;
y_truck = (y_ff + y_fr)/2;

x_trailer = (x_rf + x_rr)/2;
y_trailer = (y_rf + y_rr)/2;

if i == 1
    steer_angle = 0;
else
    steer_angle = u_cl(i,1);
end

hinch_x = x_f;
hinch_y = y_f;

truck = drawtruck(x_truck,y_truck,truck_yaw,steer_angle,blength_truck ,bwidth);
trailer = drawtrailer(x_trailer,y_trailer,trailer_yaw,steer_angle,blength_trailer,bwidth);

for j = 1:1:N
reference_x_f(j) = reference_matrix(i,j,1);
reference_y_f(j) = reference_matrix(i,j,2);
end

actual_x = [actual_x,x_f];
actual_y = [actual_y,y_f];

plot(truck(:,1), truck(:,2), 'b');
hold on;
plot(hinch_x, hinch_y, 'b--.');
hold on;
plot(trailer(:,1), trailer(:,2), 'b');
hold on;
plot(reference_x_f,reference_y_f,'r-.','linewidth',0.5);
hold on;
plot(actual_x,actual_y,'b-.','linewidth',0.5);
hold off;

axis('equal')
% axis([-10 180 -2 6])%lane change
% axis([-100 5 -6 6])%backward
axis([0 200 0 8])%circle
% axis([-10 180 -5 5])%sine
F(i) = getframe(gcf);
drawnow
end


% writerObj = VideoWriter('myVideo.avi');
% writerObj.FrameRate = 30;
% % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);