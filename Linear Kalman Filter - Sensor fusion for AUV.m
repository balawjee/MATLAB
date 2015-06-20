clear all;
clc;

%load AUV data set
load('noisyMeasOut.mat');
load('simout.mat');

% % Extract simulated sensors measurement data from Matlab workspace
tm=noisyMeasOut.time;                    % Time (s)

% 3D position from pos fix (m) - 1 Hz rate
Xm=noisyMeasOut.signals.values(:,1);     %displacement in x-axis
Ym=noisyMeasOut.signals.values(:,2);     %displacement in y-axis
Zm=noisyMeasOut.signals.values(:,3);     %displacement in z-axis

% Body velocities from DVL (m/s) at 5Hz rate - ignore column no 7, 8 and 9
um=noisyMeasOut.signals.values(:,4);     %velocity in x direction
vm=noisyMeasOut.signals.values(:,5);     %velocity in y direction
wm=noisyMeasOut.signals.values(:,6);     %velocity in z direction

% Euler angles (deg) at 20Hz rate - orientation of the vehicle
phim=noisyMeasOut.signals.values(:,10);    %phi
thetam=noisyMeasOut.signals.values(:,11);  %theta
headm=noisyMeasOut.signals.values(:,12);   

% Body angular rates (deg/s) at 20Hz rate
pm=noisyMeasOut.signals.values(:,13);   %angular velocity -> x-axis
qm=noisyMeasOut.signals.values(:,14);   %angular velocity -> y-axis
rm=noisyMeasOut.signals.values(:,15);   %angular velocity -> z-axis

% Body acceleration (m/s2) at 20 Hz rate
axm=noisyMeasOut.signals.values(:,16);  %acceleration in x-axis   
aym=noisyMeasOut.signals.values(:,17);  %acceleration in y-axis
azm=noisyMeasOut.signals.values(:,18);  %acceleration in z-axis

%x_hat is the state vector.Initially it is all zeros
x_hat=zeros(15,1);

%assuming the vehicle is at the origin of whatever reference system we are
%considering. So the vehicle starts at (0,0)
previous_pos=[0 0];

for i=1:length(noisyMeasOut.time)
    
    %initial position is (0,0)
    phi=x_hat(7);
    theta=x_hat(8);
    psi=x_hat(9);
    
    %transformation matrices - body frame to NED frame
    C_bodyrate2eulerdot  = [1      0    theta; ...
        0           1                -phi    ; ...
        0      phi    1];
    
    C_b2ned=[1 -psi theta;
        psi 1 -phi;
        -theta phi 1];
    C_ned2b=transpose(C_b2ned);
    
    %convert some measurements from body to ned frame to calculate error
    %and kalman gain in filtering part    
    ymeas(i,:)=[noisyMeasOut.signals.values(i,1:3) noisyMeasOut.signals.values(i,4:6) deg2rad(noisyMeasOut.signals.values(i,10:12)) zeros(1,6)];
    
    
    g=9.81; %acceleration due to gravity
    
    %some constants
    dtv=1/5; %doppler velocity is 5 Hz. Hence dtv=1/5
    dt=1/20; %accelerometer gives 20Hz output. Hence dt=1/20
    dt2=dt*dt/2; 
    
    %F -> state transition matrix
    %This is the "model of the system". Without this model, the kalman filter won't work. Accuracy of results depends
    %on the accuracy of the model
    F=[eye(3) C_b2ned*dt zeros(3,3) -C_b2ned*dt2 zeros(3,3);
        zeros(3,3) eye(3) zeros(3,3) -eye(3)*dt zeros(3,3);
        zeros(3,6) eye(3) zeros(3,3) -C_bodyrate2eulerdot*dt;
        zeros(6,9) eye(6)];
    
    %acc gyro
    gamma=[C_b2ned*dt2 zeros(3,3);
        eye(3)*dt zeros(3,3);
        zeros(3,3) C_bodyrate2eulerdot*dt;
        zeros(6,6);];
    
    if i==1
        %u -> input vector
        u=[noisyMeasOut.signals.values(:,16:18) deg2rad(noisyMeasOut.signals.values(:,13:15))];
        x_hat=zeros(15,1);
        h=eye(15);
                
        pos_cov=[10 5];
        height_cov=3;
        vel_cov=[2 3 1];
        angles_cov=[10 10 15];
        accbias_cov=[100 120 100];
        gyrobias_cov=[20 20 30];
        
        %P -> posteriori error covariance matrix. Initially we make a lot of assumptions.
        %this matrix specifies how much error you think exists, in the
        %assumptions you made        
        P=diag([pos_cov height_cov vel_cov angles_cov accbias_cov gyrobias_cov]);
        
        %gaussian noise
        pos_noise=[0.00015 0.00015];
        height_noise=1e-4;
        vel_noise=[0.008 0.006 0.012];
        angles_noise=0.003;
        acc_noise=[1e-10 1e-12 1e-10];
        gyro_noise=[1e-8 1e-8 0.5e-8];
        
        %q-> process noise covariance. This matrix tells how accurate the
        %model F is
        q=diag([pos_noise height_noise vel_noise ones(1,3)*angles_noise acc_noise gyro_noise]);
        
        %measurement noise
        pos_meas=[20 55];
        height_meas=6;
        vel_meas=[3.65 2.25 0.85];
        angles_meas=[2.2 2.2 1.2];%[10.2 10.2 10.2];
        accbias_meas=[120 150 120];
        gyrobias_meas=[20 20 20];
        
        %r -> measurement noise covariance. This matrix tells how accurate
        %the sensor outputs are
        r=diag([pos_meas height_meas vel_meas angles_meas accbias_meas gyrobias_meas]);
        
    end
    
    %prediction step - Kalman filter makes a prediction based on the model
    %F, initial values x_hat and the error covariances q and P
    x_hat=F*x_hat + gamma*u(i,:)';
    P=F*P*(F') + q;
    
    pos_prediction=x_hat(1:2);
    P_pos=P(1:2,1:2);
    if(i>1)
        k_pos=k(1:2);
    end
    
    if previous_pos(1)==ymeas(i,1) && previous_pos(2)==ymeas(i,2)
        GPS_valid=0;
    else
        GPS_valid=1;
    end
    
    %kalman gain 
    k=P*(h')*inv(h*P*(h') + r);
    
    %correction/updation step - kalman filter's prediciton is compared with
    %the actual sensor output. Based on error (sensor output - prediction),
    %the kalman filter updates the values P and kalman gain. This process
    %is repeated for every timestep. So if the Kalman filter is properly
    %tuned, it gets better and better with every prediction. The efficacy
    %of prediction depends on model F, and initial values assigned for P,q
    %and r. Unforutanely there is no standard way to tell how to set the values of
    %P,q,r
    innov=ymeas(i,:)'-h*x_hat;
    in(i,:)=innov;
    x_hat=x_hat+k*(innov);
    correction(i,:)=x_hat;
    op(i,:)=x_hat;
    P=(eye(length(x_hat))-k*h)*P;
    P_hist(i,:)=diag(P);
    
    gps(i)=GPS_valid;
    %     %if new GPS is not available, retain prediciton and ignore the
    %     %correction for GPS pos alone
    if GPS_valid==0
        x_hat(1)=pos_prediction(1);
        x_hat(2)=pos_prediction(2);
        P(1:2,1:2)=P_pos;
        k(1:2)=k_pos;
        op(i,:)=x_hat;
        P_hist(i,:)=diag(P);
    end
    
    kalman_gain(i,:)=diag(k);
    %store previous position to check if location fix is available for next
    %iteration
    previous_pos=ymeas(i,1:2);
end

for j=10:15
    disp(mean(op(:,j)));
end

%plotting the outputs
figure;
plot(tm,simout.signals.values(:,1),'b-.');hold on;
plot(tm,op(:,1),'r-.');
hold on;plot(tm,Xm,'k');
xlabel('time');

figure;
plot(tm,simout.signals.values(:,2),'b-.');hold on;
plot(tm,op(:,2),'r-.');
hold on;plot(tm,Ym,'k');
xlabel('time');
