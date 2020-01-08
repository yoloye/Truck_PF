function [x_ffp, y_ffp, x_frp, y_frp, x_rfp, y_rfp, x_rrp, y_rrp] = position_calculation(theta_truck, theta_tralior, x_fp,y_fp,l_fa, l_fb, l_fc, l_ra, l_rb, l_rc)
truck_rot_matrix = rot2D(theta_truck);
tralior_rot_matrix = rot2D(theta_tralior);
position_ff = [(l_fa+l_fb), 0];
position_ff_rot = (truck_rot_matrix'*position_ff')';
position_fr = [-(l_fc), 0];
position_fr_rot = (truck_rot_matrix'*position_fr')';
position_rf = [(l_ra), 0];
position_rf_rot = (tralior_rot_matrix'*position_rf')';
position_rr = [-(l_rb+l_rc), 0];
position_rr_rot = (tralior_rot_matrix'*position_rr')';
x_ffp = x_fp+position_ff_rot(:,1);
y_ffp = y_fp+position_ff_rot(:,2);
x_frp = x_fp+position_fr_rot(1,1);
y_frp = y_fp+position_fr_rot(1,2);
x_rfp = x_fp+position_rf_rot(1,1);
y_rfp = y_fp+position_rf_rot(1,2);
x_rrp = x_fp+position_rr_rot(1,1); 
y_rrp = y_fp+position_rr_rot(1,2);
end
