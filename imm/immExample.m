clear all;
iMC=10;
modepron=[];
x=[];
for i=1:iMC
    %% Model 1: CV-Model
    modelCV=CV(5,1);
    %% Model 2: CA-Model
    modelCA=CA(5,15);
    %% ground thruth
    immgt=IMMGroundTruthGenerator(200,modelCV,modelCA);
    % Forced mode transitions 1=150 2=50
    immgt.mode(1:50) = 1;
    immgt.mode(51:70) = 2;
    immgt.mode(71:120) = 1;
    immgt.mode(121:150) = 2;
    immgt.mode(151:200) = 1;
    immgt.initInitialState([0;0;0]);
    immgt.generateGroundTruth();
    measurementModels={[1 0],[1 0 0]};
    immgt.generateMeasurements(measurementModels);
    
    
    
    %% Setup Kalman Filter 1
    kf1=KalmanFilter(modelCV);
    x0=[0;0];
    % configure start values
    kf1.initInitialState(x0);
    kf1.initInitialCovariance(eye(2));
    % configure measurement model
    kf1.setMeasurmentModel('s');
    kf1.setMeasurmentCovariance(eye(1));
    %kf1.run(immgt.measurements,0);
    %x=kf1.getState();
    %plot(x(1,:)-immgt.x(1,:),'--');
    %plot(x(1,:)-immgt.measurements,'--');
    %% Setup Kalman Filter 2
    kf2=KalmanFilter(modelCA);
    x0=[0;0;0];
    % configure start values
    kf2.initInitialState(x0);
    kf2.initInitialCovariance(eye(3));
    % configure measurement model
    kf2.setMeasurmentModel('s');
    kf2.setMeasurmentCovariance(eye(1));
    %kf2.run(immgt.measurements,0);
    %x2=kf2.getState();
    %plot(x2(3,:)-immgt.x(3,:),'--');
    %plot(x2(1,:)-immgt.measurements,'--');
    %plot(x2(1,:)-immgt.measurements,'r');hold on;plot(x(1,:)-immgt.measurements,'--');
    %plot(x2(2,:)-x(2,:));
    %% Setup IMM
    imm=IMM(kf1,kf2);
    imm.measurements=immgt.measurements;
    imm.p_ji=[0.25 0.75;
        0.25 0.75];
    imm.setInitialModeProbability([0.5;0.5]);
    imm.run();
    
    modepron=[modepron;imm.modeProbability(2,:)];
    x=[x;imm.x(1,:)-immgt.x(1,:)];
    
    for j=1:200
        Filter1(i,j)=imm.filterX{j}{1}(1)-immgt.x(1,j);
    end
    
        for j=1:200
        Filter2(i,j)=imm.filterX{j}{2}(1)-immgt.x(1,j);
        end
    
        %%NEES
        for j=1:200
        IMMNEES(j)=(imm.x(:,j)-immgt.x(:,j))'*pinv(imm.P{j})*(imm.x(:,j)-immgt.x(:,j));
        end
end


plot(sum(modepron,1)/iMC)

%% RMSE
RMSEIMM=sqrt(sum(x.^2)/iMC);
RMSEFilter1=sqrt(sum(Filter1).^2/iMC);
RMSEFilter2=sqrt(sum(Filter2.^2)/iMC);

plot(RMSEIMM,'r');
hold on;
plot(RMSEFilter1,'g');
hold on;
plot(RMSEFilter2);

csvwrite('RMSEIMM.csv',[(1:200);RMSEIMM]');
csvwrite('RMSEFilter1.csv',[(1:200);RMSEFilter1]');
csvwrite('RMSEFilter2.csv',[(1:200);RMSEFilter2]');

%% NEES
plot(IMMNEES);
hold on;
plot();



%% Visualization
immgt.plotModeProbability();
hold on;
plot(imm.modeProbability(2,:));

csvwrite('modeProbability.csv',[(1:200);sum(modepron,1)/iMC]')

csvwrite('mode.csv',[(1:200);immgt.mode-1]')

imm.modeProbability(2,:)
immgt.plotMeasurements();
immgt.plotGroundTruth();

imm.plotEstimation();
imm.plotFilterEstimation();

imm.plotModelLikelihood(1);
hold on;
imm.plotModelLikelihood(2);
imm.plotCovariance(1,1);
imm.plotResidualCovariance(1);
hold on;
imm.plotResidualCovariance(2);

x1=imm.getFilterEstimation(1);
plot(x1(1,:));
hold on;
x2=imm.getFilterEstimation(2);
plot(x2(1,:));
hold on;
immgt.plotMeasurements();
hold on;
immgt.plotState(1);


plot(imm.z)