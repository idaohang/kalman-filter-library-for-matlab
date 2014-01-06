%% Description
% Some Changes
% 1. Not a function
% 2. Un/Comment all graphical Output
%% NEES OBJ
NEESOBJ=NEES();
%%
% no. of time steps
K  = 150;

% constants, cf. lecture slides
k  = 5;     % in N/m
D  = 1;     % in Ns/m
T  = 0.1;   % in s
m  = 0.5;   % in kg
Fe = 1;     % in N
g  = 9.81;  % m/s²

% State transition 
A = [0 1;
    -(1+(k*(T^2))/m-(D*T)/m) 2-(D*T)/m];

% Transition of control input
B = [0 0;
     0 1];

% Observation Matrix
H = [1 0];
 
Cxe = 0.15*eye(2,2); % Uncertainty about inital state.
Cu  = (0.05)^2*[1 0; 0 1]; %  Uncertainty of excitation.
Cy  = (0.05)^2; % Uncertainty of observation.

% Starting values
x = zeros(2,K);
x_no = zeros(2,K);
xp = x;
x(:,1) = [m*g/k m*g/k]';
y  =zeros(1,K);
xe = zeros(2,K);

% System input
u = zeros(2,K);
u_no  = zeros(2,K);
u(2,:)= ones(1,K)*g*(T^2);
u_no(2,:)= ones(1,K)*g*(T^2);
for i = 1:K % Noisy sine excitation
    u(2,i) = ((Fe*(T^2))/m)*sin(i*T*2*pi*sqrt(m/k)) + 1*randn;
    u_no(2,i) = ((Fe*(T^2))/m)*sin(i*T*2*pi*sqrt(m/k));
end
Cu  = var(u(2,:))*[1 0; 0 1];
Cu  = [-10/3+18/2-9 1; -20/4+36/3-18/2 4/3];
% Uncomment to see what happens in the case of no excitation?
%u(2,50:100)=0;

% Graphical output of input
% fig = figure;
% set(fig,'DoubleBuffer','on');
% subplot(3,1,1);
% plot(u(2,:));
% xlabel(strcat('u_{1:',int2str(K),'}'));
% title('Input')
% waitforbuttonpress


% Sigma Bounds
so = zeros(1,K);
so(1)=sqrt(Cxe(1,1));
su = zeros(1,K);
su(1)=-sqrt(Cxe(1,1));

%% Calculate evolution of system dynamics and save it as true state
for i=2:K
    % Evolution of system dynamics 
    x(:,i) = A*(x(:,i-1))+B*u(:,i-1);
    x_no(:,i) = A*(x_no(:,i-1))+B*u_no(:,i-1); 
end
NEESOBJ.setTrueState(x);

for iMC=1:100
% Starting values for every MC-Run
y  =zeros(1,K);
xe = zeros(2,K);
xe(:,1)=[m*g/k m*g/k]';
Cxe = 0.15*eye(2,2); % Uncertainty about inital state.
P{1}= Cxe;
for i=2:K
    % Observation
    y(i) = x(1,i)+0.05*randn;
    
    % Plot observation
%     subplot(3,1,2);plot([1:i],y(1:i)-(m*g/k),'r-');
%     axis([1 K -1.5 0]);
%     title('Noisy observation');
%     ylabel(strcat('x')); 
%     xlabel(strcat('x_k, k = 1:',int2str(i)));

    % Prediction step
    xp(:,i) = A*xe(:,i-1) + B*u(:,i-1);
    Cxp=A*Cxe*A' + B*Cu*B';

    % Filter step
    Kg = Cxp*H'/(Cy+H*Cxp*H');
    xe(:,i) = xp(:,i)+Kg*(y(i)-H*xp(:,i));
    Cxe = Cxp - Cxp*H'/(Cy+H*Cxp*H')*H*Cxp;
    % For NEES save Cxe as Struct
    P{i}=Cxe;
    % Graphical output of estimate
%     s = 1*sqrt(Cxe(1,1)); % Calculate sigma bound
%     so(i) = xe(1,i)+*s;
%     su(i)=  xe(1,i)-*s;
%     subplot(3,1,3);plot([1:i],xe(1,1:i)-(m*g/k),'b-');
%     axis([1 K -1.5 0]); 
%     hold on
%     subplot(3,1,3);plot([1:i],so(1:i)-(m*g/k),'r--');
%     subplot(3,1,3);plot([1:i],su(1:i)-(m*g/k),'r--');
%     xlabel('x_e \pm s\sigma'); title('State estimate');
%     drawnow
end
    NEESOBJ.addEstimatedState(xe);
    NEESOBJ.addEstimatedCovariance(P);
end
NEESOBJ.calculateNEES();
NEESOBJ.plotNEES();


% for i=1:150
% var_true=cov(NEESOBJ.x_sch(1,:),NEESOBJ.x_sch(2,:)');
% var(i)=P{i}(2,2);
% plot(var);
% hold on;
% plot(var_true(2,2)*ones(1,150));
% end