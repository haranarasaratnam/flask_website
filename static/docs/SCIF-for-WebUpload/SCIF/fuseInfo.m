function  [ykk,Sykk] = fuseInfo(iArray, SIArray,ykk1,Sykk1)

ykk = ykk1 + sum(iArray,2);

[foo,Sykk] = qr([Sykk1 SIArray]',0);

Sykk = Sykk';
