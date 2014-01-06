syms Ts;
syms tau;
k=1;
m=1;
D=1;
Ts=1-tau;
A = [0 1; -(1+(k*Ts^2/m)-D*Ts/m) (2-D*Ts/m)];


A*[0;1]*[0 1]*A


A=[-10/3+18/2-9 1; -20/4+36/3-18/2 4/3];


syms Ts;
syms k;
syms D;
syms m;
A = [0 1; -k/m -D/m];
simplify(expm(Ts*A))