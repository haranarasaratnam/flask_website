function z = MstEq(x,s)

r = sqrt(  (x(1,:)-s(1)).^2 + (x(3,:)-s(2)).^2 );

rdot = ( (x(1,:)-s(1)).*x(2,:) + (x(3,:)-s(2)).*x(4,:) )./r;

z = [r; rdot];