function [y,Sy] = Convert_state_2_info(x,Sx)

Sy = eye(5)/Sx';

% Sy = pinv(Sx');

y = Sy*Sy'*x;