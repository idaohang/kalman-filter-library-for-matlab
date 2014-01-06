%% Kalman-Filter Instanz
KF=KalmanFilter();
T=1;
KF.F=[1 T; 0 1];
KF.H=[1 0];
KF.T=T;
scalarnoise=randn(1,100);
KF.Q=[0.25*T^4 0.5*T^3; 0.5*T^3 T^2]*var(scalarnoise);
KF.Q=cov([0.5*T^2*scalarnoise;T*scalarnoise],[0.5*T^2*scalarnoise;T*scalarnoise]');
KF.Q=[1 1;1 1]*var(scalarnoise);
KF.R=1;
KF.G=0;
KF.x=[0 0]';
KF.P=[1 1; 1 1];
%% GroundThruth
trueState=[0;0];
for i=2:100
    trueState(:,i)=KF.F*trueState(:,i-1)+scalarnoise(i);
end
%% NEES Instanz
NEESOBJ=NEES();
RMSEOBJ=RMSE();
NEESOBJ.setTrueState([0.5*T^2*scalarnoise;T*scalarnoise]);
NEESOBJ.setTrueState([zeros(1,100);zeros(1,100)]);
NEESOBJ.setTrueState(trueState);
RMSEOBJ.setTrueState([0.5*scalarnoise;scalarnoise]);
%% Monte-Carlo-Runs
for iMCR=1:500
    Measurements=sqrt(KF.R)*randn(1,100)+trueState(1,:);
    KF.run(Measurements,Measurements);    
    KF.x=[0 0]';
    KF.P=[1 1; 1 1];
    NEESOBJ.addEstimatedState(KF.getState(0));
    NEESOBJ.addEstimatedCovariance(KF.getCovariance(0,0));
    RMSEOBJ.addEstimatedState(KF.getState(0));
end
NEESOBJ.calculateNEES();
NEESOBJ.plotNEES();
RMSEOBJ.calculateRMSE(1);


