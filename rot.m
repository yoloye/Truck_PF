function R_matrix=rot(angle, axis)
% R=rot(angle (rad), axis) returns a 3x3
% rotation matrix for rotating a vector about a single
% axis.  Setting axis = 1 rotates about the e1 axis,
% axis = 2 rotates about the e2 axis, axis = 3 rotates
% about the e3 axis.  Uses right handed coordinates, so
% rotations are counterclockwise about axis.

R_matrix=eye(3);
cang=cos(angle);
sang=sin(angle);

if (axis==1)
R_matrix(2,2)=cang;
R_matrix(2,3)=sang;
R_matrix(3,2)=-sang;
R_matrix(3,3)=cang;
end;

if (axis==2)
R_matrix(1,1)=cang;
R_matrix(1,3)=-sang;
R_matrix(3,1)=sang;
R_matrix(3,3)=cang;
end;

if (axis==3)
R_matrix(1,1)=cang;
R_matrix(1,2)=sang;
R_matrix(2,1)=-sang;
R_matrix(2,2)=cang;
end;

return;
end