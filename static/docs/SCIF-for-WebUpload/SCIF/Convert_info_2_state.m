function [x,Sx] = Convert_info_2_state(y,Sy)

Sx = eye(5)/Sy';

x = Sx*Sx'*y;