
function [i,SI] = Update(ykk1,Sykk1,xkk1,Skk1,z,s)

global Rsqrt Rsqrt_inv QPtArray  nPts nx nz;

Xi =  repmat(xkk1,1,nPts) + Skk1*QPtArray;
    
Zi = MstEq(Xi,s);
    
zkk1 = sum(Zi,2)/nPts; 

X = (Xi-repmat(xkk1,1,nPts))/sqrt(nPts);
    
Z = (Zi-repmat(zkk1,1,nPts))/sqrt(nPts);  

[foo,S] = qr([Z Rsqrt; X zeros(nx,nz)]',0);

S = S';

X = S(1:nz,1:nz);

Y = S(nz+1:end,1:nz);

SI = Sykk1*Sykk1'*(Y*X')*Rsqrt_inv;  %u matrix

i = (SI*Rsqrt_inv')*(z-zkk1)  +  SI*SI'*xkk1;



% [foo,Sykk] = qr([Sykk1 SI]',0);
% 
% Sykk = Sykk';
% 
% ykk = ykk1 + i;
