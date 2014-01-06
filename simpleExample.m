clear all;
%% System
x=[0;0]; % starting point
T=0.5;
F=[1 T; 0 1];
H=[1 0];
for i=2:100
    x(:,i)=F*x(:,i-1)+randn;
end
% measurments
z=H*x(:,1:end);
%% Setup
KF=KalmanFilter();
KF.F=F;
KF.H=H;
KF.T=T;
KF.Q=[1 0; 0 1];
KF.R=1;
KF.G=0;
KF.x=[0 0]';
KF.P=[1 1; 1 1];
%% Estimation
KF.run(z,0);
