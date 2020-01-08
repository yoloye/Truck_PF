function R_matrix = rot2D(ang)
% Standard 2D rotation using 3D function
R_matrix = rot(ang,3);
R_matrix = R_matrix(1:2,1:2);
end