%% temp
clear all;

%% Example How to use modelLib with Kalman-Filter class
% Setup a CV model with a sampel rate of 5
modelCV=CV(5);
% Setup a state variable configuration
modelCV.setStateTransitionMatrix('s','s_x','s','s_y','s','s_z','v','v_x','v','v_y','v','v_z');

%% Setup Kalman Filter
kf=KalmanFilter(modelCV);
x0=[0;0;0;0;0];
% configure start values
kf.initInitalState(x0);
kf.initInitalCovariance(eye(6));
% configure measurement model
kf.setMeasurmentModel('s_x','s_y','s_z');
kf.setMeasurmentCovar(eye(3));
%% Setup GroundTruthGenerator
gt=GroundTruthGenerator(modelCV,100);
gt.initInitalState(x0);
gt.generateGroundTruth(kf.Q);
gt.generateMeasurements(kf.H,kf.R);

%% Start Estimation
kf.setMeasurments(gt.measurements);
kf.run();