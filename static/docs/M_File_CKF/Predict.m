function [xkk1,Pkk1] = Predict(xkk,Pkk)

global Q;

xkk1 = xkk;

Pkk1 = Pkk + Q;


