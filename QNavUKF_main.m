%clc
%clear all
%close all
clc;clear;close all;
% S_temp = load('data_newgood_repeatable.mat');
% rng(S_temp.seed_data.seed_val)

seed_data = load('seed_data_keep.mat');
rng(seed_data.seed_val)

% seed_val = rng;

tic

% Navigation on SE_2(3) with biased angular velocity
% Limited number of featrues in the pointTracker!
% for example 30. This method saves time for the feature tracking
%% Read dataset

% Load stereo camera intrinsic parameters (calibrated using MATLAB)
load('stereoParamsdataset.mat'); 

% Set the path of different experiments
setpath = 2;
switch  setpath
    case 1
        path        = 'C:\Offline\Datasets\V1_01_easy\mav0'; 
    case 2
        path        = 'C:\Offline\Datasets\V1_02_medium\mav0'; 
    case 3
        path        = 'V2_01_easy'; 
    case 4
        path        = 'V2_02_medium'; 
end

% Read the dataset
[IMURead, ViconRead, Groundtruth, CamRead]= dataRead(path); 
bw        = mean(Groundtruth(:,12:14)); %averaged gyro bias from groundtruth
ba        = mean(Groundtruth(end-100:end,15:17)); %averaged accel bias from groundtruth
TimerIMU  = IMURead(:,1)*1e-9;  %nanoseconds to seconds 
TimerCam  = CamRead(:,1)*1e-9;  
% transfromation from body frame to cam0 frame 
TB2C0 = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
          0.0, 0.0, 0.0, 1.0];
 
%% Keyframe initialization
numFeatures = 60;
index       = 1;
[imageLeft,imageRight]                       = stereoRead(path,CamRead,index,stereoParams);
% pointTracker = vision.PointTracker;
pointTracker = vision.PointTracker('BlockSize',[9 9],'MaxBidirectionalError',1); %Reduce the blocksize can improve the accuracy
[pointsKeyR,pointsKeyL,pointTracker] = funPointTracker(imageLeft,imageRight,numFeatures,pointTracker);
% Show matched features
% figure; ax = axes;
% showMatchedFeatures(imageLeft,imageRight,pointsKeyL,pointsKeyR,'blend','Parent',ax);
% % title(ax, 'Candidate point matches');
% legend(ax, 'Matched points left','Matched points right');
% set(gcf, 'Renderer', 'Painters');
% print('-depsc','Features.eps')
%% Initialize observer

g  = 9.81;
e3 = [0,0,1]';
 
% P  = 0.1*eye(6); 
% kR = 0.15;
% kv = 0.5;
% kp = 0.5;
% kw = 30;
% Q  = 20*eye(3);
% V  = 10*diag([1*ones(1,3) 1*ones(1,3)]); 
    
    
p  = Groundtruth(1,2:4)';
R  = quat2rotm(Groundtruth(1,5:8));
v  = Groundtruth(1,9:11)';
 



Rkey = R;
pkey = p;
vkey = v; 
points3D  = funPoints3D(pointsKeyL,pointsKeyR,stereoParams,TB2C0);
pointsKey = Rkey*points3D+pkey;  %inertial frame landmarks  

u      = e3;%randn(3,1);
theta  = 0.99*pi;%(0.5-rand(1))*0.4*pi;
Rq     = expm(Skew(u)*theta);
% Xhat   = [Rq*R 0*v+0.0*randn(3,1) 0*p+0.0*randn(3,1);
%          zeros(2,3) eye(2)];

% Xhat   = [rotm2quat(Rq*R)' ; 0*p+0.0*randn(3,1) ; 0*v+0.0*randn(3,1) ];
% Xhat   = [Rq*R 1*v+0.0*randn(3,1) 1*p+0.0*randn(3,1);
%          zeros(2,3) eye(2)];
% Xhat   = [eye(3) 0*v 0*p;zeros(2,3) eye(2)];

bwhat  = zeros(3,1);

bwhat2 = bwhat;

Tfinal     = ceil(length(TimerIMU)*0.10);%20000;%
Xhat_log = zeros(16 , Tfinal);
% Xhat = [rotm2quat(Rq*R)' ; 0*p+0.0*randn(3,1) ;...
%     0*v+0.0*randn(3,1);bw'*0;ba'*0];
qhat = rotm2quat(R)';
% qhat = qhat+5.110*qhat;
qhat = 2*qhat+8*[2 , 3 , -2 , 1]';
qhat = qhat/norm(qhat);
Xhat = [qhat ; 1.00*p+7.0*[1, 1, -2]' ;...
    0*v+0.0*randn(3,1);bw'*1;ba'*1];
Xhat_log(: , 1) = Xhat;


dim_x = 16;
dim_z = [];
dt=1/200;
alpha = 1e-4;
beta = 2;
kappa = 0;
sigma_mode = 2;
augmentation = true;
dim_xa = 16+  6;
xa = zeros(6 , 1);
% xa = [bw' ; ba'];
q_indices = {[1 , 2 , 3 , 4]};
P_q_indices = {[1 , 2 , 3]};
[state, params] = initialize_ukf(dim_x, dim_z, dt, alpha, beta, kappa,...
    sigma_mode, augmentation, dim_xa, xa, q_indices, P_q_indices);
state.x = zeros(dim_x, 1);  % State vector
% state.P = eye(dim_x-1)*10^2;       % State covariance matrix
% state.P = blkdiag(0.01^2*ones(3)*0 , zeros(3) , 10^2*ones(3) , eye(6)*10);
P0_ = blkdiag(zeros(3)+eye(3)*10^3*5 , zeros(3)+eye(3)*6*10^2 , 1*10^1*eye(3) , eye(6)*10^1);
state.P = P0_;

params.Q = zeros(dim_x-1);       
params.Q(10:12 , 10:12) = diag(bw.^2)*10^-8;
params.Q(13:15 , 13:15) = diag(ba.^2)*10^-8;
omega_mean = mean(IMURead(:,2:4)) ;
a_mean     = mean(IMURead(:,5:7)) ;
params.Qa = blkdiag(diag(omega_mean.^2)*10^-4 , diag(a_mean.^2)*10^-4);
% params.Qa = blkdiag(sqrtm(diag(omega_mean))*10^-4 , diag(a_mean)*10^-4);
% params.R = (0.099538^2)*eye(dim_z);       % Measurement noise matrix
state.x = Xhat;

% state_right = state;
% params_right = params;
% params_right.Qa = blkdiag(sqrtm(diag(omega_mean))*10^-4 , abs(diag(a_mean))*10^-4);


Time = (TimerIMU(1:Tfinal)-TimerIMU(1));
%% Loop

Pestimate  = zeros(3,Tfinal);
Pestimate2 = zeros(3,Tfinal); 
normR      = zeros(2,Tfinal);
normV      = zeros(2,Tfinal); 
normbw     = zeros(2,Tfinal);
deul = zeros(3 , Tfinal);
drotv = zeros(3  , Tfinal);
dtheta = zeros(1 , Tfinal);
% update_time = zeros(1 , Tfinal);
update_state = [];
update_time = [];
y_e = {};
dim_tot_ = 0;
Numlandmark    = zeros(size(CamRead));
keytime        = zeros(size(CamRead));
keytime(index) = 1;
for k=1:1:Tfinal
    if mod(floor(100*k/Tfinal), 5) == 0
        clc
        percent_complete = floor(100 * k / Tfinal);
        elapsed_time = toc; % Measure the elapsed time
        
        % Estimate the total time based on current progress
        estimated_total_time = elapsed_time / (k / (Tfinal - 1));
        estimated_time_left = estimated_total_time - elapsed_time;
        
        % Convert elapsed_time to hours, minutes, and seconds
        elapsed_hours = floor(elapsed_time / 3600);
        elapsed_minutes = floor(mod(elapsed_time, 3600) / 60);
        elapsed_seconds = round(mod(elapsed_time, 60)); % Round to nearest second
        
        % Convert estimated_time_left to hours, minutes, and seconds
        estimated_hours = floor(estimated_time_left / 3600);
        estimated_minutes = floor(mod(estimated_time_left, 3600) / 60);
        estimated_seconds = round(mod(estimated_time_left, 60)); % Round to nearest second
        
        % Display the progress and estimated time left
        disp(['Complete  ' num2str(percent_complete) '%']);
        disp(['Elapsed time: ' sprintf('%02d:%02d:%02d', elapsed_hours, elapsed_minutes, elapsed_seconds)]);
        disp(['Estimated time left: ' sprintf('%02d:%02d:%02d', estimated_hours, estimated_minutes, estimated_seconds)]);
    end
    % errors
    p       = Groundtruth(k,2:4)';
    R       = quat2rotm(Groundtruth(k,5:8));
    v       = Groundtruth(k,9:11)';     
    normR(1,k)      = trace(eye(3)-R*quat2rotm(Xhat(1:4)')')/4; 
    daxang = rotm2axang(R*quat2rotm(Xhat(1:4)')');
    dtheta(k) = rad2deg(daxang(4));
    deul(: , k) = rotm2eul(R)' - quat2eul(Xhat(1:4)')';
    deul(: , k) = wrapTo180(rad2deg(deul(: , k)));
    drotv(: , k) = qMq(rotm2quat(R)' , Xhat(1:4));
    normV(1,k)      = norm(v-Xhat(8:10)); 
    normbw(1,k)     = norm(bw'-Xhat(11:13));
    Pestimate(:,k)  = Xhat(5:7)'; 
    Xhat_log(: , k) = Xhat;
%         propagation        
    dT    = TimerIMU(k+1)-TimerIMU(k); 
    omega = IMURead(k,2:4)' ;
    a     = IMURead(k,5:7)' ;

    params.dt = dT;
    params_right.dt = dT;
    [state, params] = predict(state, params  , omega , a , g , e3);
%     disp(state.x')
%     [state_right, params_right] = predict(state_right, params_right  , omega , a , g , e3);
%     Xhat = state.x;
    if TimerIMU(k+1)>= TimerCam(index+1,1)
        index = index +1;               
        [imageLeft,imageRight]             = stereoRead(path,CamRead,index,stereoParams);
        [pointsNewL,pointsNewR,isFoundand] = stereoPointTrack(imageLeft,imageRight,pointTracker);        
        % tracked features locations
        pointsNewR = pointsNewR(isFoundand,:);
        pointsNewL = pointsNewL(isFoundand,:);
%         pointsNewL = pointsNewL + 1*randn(size(pointsNewL)); %add noise for analysis
%         pointsNewR = pointsNewR + 1*randn(size(pointsNewR));
        
        pointsI    = pointsKey(:,isFoundand);
        pointsB   = funPoints3D(pointsNewL,pointsNewR,stereoParams,TB2C0);  
        [pointsI,pointsB,pointsNum] = pointsSelection(pointsI,pointsB); 
        Numlandmark(index)= size(pointsI,2);
%         pointsNum = size(pointsI,2);
        
        if (pointsNum>=3)
            z_std = 0; 
%             params.R = (z_std+0.098898)^2*eye(pointsNum*3);  %easy    % Measurement noise matrix
            params.R = (z_std+0.099538)^2*eye(pointsNum*3);  %medium
            params_right.R = (z_std+0.099538)^2*eye(pointsNum*3);
%             pointsB = pointsB + z_std*randn(size(pointsB));
            [state, params] = update(state, params,...
                pointsB + z_std*randn(size(pointsB)) ,...
                pointsNum , pointsI); 
%             [state_right, params_right] = update(state_right, params_right,...
%                 pointsB + z_std*randn(size(pointsB)) ,...
%                 pointsNum , pointsI); 

            p_= Groundtruth(k,2:4)';
            R_= quat2rotm(Groundtruth(k,5:8));
            y_e_ = reshape(R_'*(pointsI - p_) - pointsB , [] , 1);
%             y_e = [y_e , reshape(y_e_ , [] , 1)];
            y_e{end+1} = y_e_;
            dim_tot_ = dim_tot_ + length(y_e_);
            update_state = [update_state ;1];
            update_time = [update_time ; Time(k)];
        else
            update_state = [update_state ;0];
            update_time = [update_time ; Time(k)];
        end %endif
        % Creat new Keyframe if pointsNum is less than 6     
        minFeatures = 6;
        if pointsNum <= minFeatures %size(find(isFoundand==1), 1) < minFeatures       
            [pointsKeyR,pointsKeyL,pointTracker] = funPointTracker(imageLeft,imageRight,numFeatures,pointTracker);                 
            pkey      = Groundtruth(k,2:4)';
            Rkey      = quat2rotm(Groundtruth(k,5:8));
            points3D  = funPoints3D(pointsKeyL,pointsKeyR,stereoParams,TB2C0);
            pointsKey = Rkey*points3D+pkey;  %inertial frame landmarks 
%             path_to_save = 'C:\Offline\Matched Images\1\' + string(k)+'.png';
%             saveMatchedFeatures(imageLeft, imageRight, pointsKeyL,...
%                 pointsKeyR, path_to_save);
            keytime(index) = 1;

         end %endif
        
         
    end %endif
    
    
    Xhat = state.x;
end %endfor
 
release(pointTracker);

currentDateTime = datetime('now');

% Convert the date and time to a string in the desired format
dateTimeString = datestr(currentDateTime, 'yyyymmdd_HHMMSS');

% Create a filename with the date and time
filename = ['data_QNUKF_' dateTimeString '.mat'];

% Save the data to the .mat file
save(filename);

disp(['Data saved to file: ' filename]);

% Estimate the scaler of the covariance matrix using MLE
sigma2 = 0; % initialize the scaler
for i = 1:length (y_e) % loop over the cell array
    sigma2 = sigma2 + norm (y_e{i})^2; % update the sum of squared norms
end
sigma2 = sigma2 / dim_tot_; % divide by the total dimension
disp (['The estimated std is: ', num2str(sqrt(sigma2))]);
% clearvars -except TimerIMU Groundtruth Tfinal Pestimate Pestimate2 ...
%     Numlandmark TimerCam keytime normR normbw normV Xhat_log dtheta deul
%% Plot
PGrdtruth = Groundtruth(1:Tfinal,2:4)';
errorP     = Pestimate - PGrdtruth;
errorPnorm = sqrt(sum(errorP.^2,1));
RMSE       = sqrt(mean(errorPnorm.^2));

errorP2     = Pestimate2;
errorPnorm2 = sqrt(sum(errorP2.^2,1));
RMSE2       = sqrt(mean(errorPnorm2.^2)); 


% figure('Position',[22   562   560   420]);
figure();

subplot(2,2,1)
plot(Time,vecnorm(drotv),'r-','linewidth',1)
% plot(Time,sqrt(normR(2,:))','r-',Time,sqrt(normR(1,:))','b-','linewidth',1), grid on
% legend('NavObsv', 'NavObsv-CRE')
ylabel('$|\tilde{R}|_I$','interpreter','latex')
xlabel('$t(s)$','interpreter','latex')
xlim([0 Time(end)])
% ylim([0 1.0])
%     axes('position',[.22 .72 .20 .22])
%     box on % put box around new pair of axes
%     indexOfInterest1  = (Time>=95) & (Time<=100); 
%     plot(Time(indexOfInterest1),sqrt(normR(2,indexOfInterest1))','r-',...
%         Time(indexOfInterest1),sqrt(normR(1,indexOfInterest1))','b-','linewidth',1), grid on
%     axis tight



subplot(2,2,2) 
plot(Time,errorPnorm2,'r-',Time,errorPnorm,'b-','linewidth',1), grid on
xlim([0 Time(end)])
% ylim([0 0.9]) 
ylabel('$\|p-\hat{p}\|$','interpreter','latex')
xlabel('$t(s)$','interpreter','latex')
%     axes('position',[.72 .72 .20 .22])
%     box on % put box around new pair of axes
%     indexOfInterest1  = (Time>=95) & (Time<=100); 
%     plot(Time(indexOfInterest1), (errorPnorm2(indexOfInterest1))','r-',...
%         Time(indexOfInterest1), (errorPnorm(indexOfInterest1))','b-','linewidth',1), grid on
%     axis tight




subplot(2,2,3)
plot(Time,normV(2,:)','r-',Time,normV(1,:),'b-','linewidth',1), grid on
ylabel('$\|v-\hat{v}\|$','interpreter','latex')
xlabel('$t(s)$','interpreter','latex')
xlim([0 Time(end)])
% ylim([0 1.6]) 




subplot(2,2,4)
plot(Time,normbw(2,:)','r-',Time,normbw(1,:),'b-','linewidth',1), grid on
ylabel('$\|b_\omega-\hat{b}_\omega\|$','interpreter','latex')
xlim([0 Time(end)])
xlabel('$t(s)$','interpreter','latex')
% ylim([0 0.16]) 

 set(gcf, 'Renderer', 'Painters');
 print('-depsc','SimulationError3.eps')
 
figure
plot(TimerCam-TimerIMU(1),Numlandmark,'.'), hold on
xlim([0 Time(end)])
Keyframetime = TimerCam(keytime==1);  
plot(Keyframetime-TimerIMU(1),zeros(size(Keyframetime)),'*')

figure 
plot3(PGrdtruth(1,:), PGrdtruth(2,:),PGrdtruth(3,:),'g--','linewidth',1), hold on
% plot3(Pestimate2(1,:), Pestimate2(2,:),Pestimate2(3,:),'r-','linewidth',1) 
plot3(Pestimate(1,:), Pestimate(2,:),Pestimate(3,:),'b--','linewidth',1) 
% title(['RMSE=' num2str(RMSE)])
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
grid on
legend('True trajectory', 'UKF')
 set(gcf, 'Renderer', 'Painters');
 print('-depsc','3dTrajectory3.eps')

% Plotting X, Y, and Z components separately
figure('Position',[22   562   560   420]);
subplot(3,1,1); % X component
plot(Time , PGrdtruth(1,:), 'r-','linewidth',1), hold on;
plot(Time , Pestimate(1,:), 'b--','linewidth',1);
title('X Component');
xlabel('Time');
ylabel('X');
legend('Ground Truth', 'Estimate');
grid on;
hold off;

subplot(3,1,2); % Y component
plot(Time , PGrdtruth(2,:), 'r-','linewidth',1), hold on;
plot(Time , Pestimate(2,:), 'b--','linewidth',1);
title('Y Component');
xlabel('Time');
ylabel('Y');
legend('Ground Truth', 'Estimate');
grid on;
hold off;

subplot(3,1,3); % Z component
plot(Time , PGrdtruth(3,:), 'r-','linewidth',1), hold on;
plot(Time , Pestimate(3,:), 'b--','linewidth',1);
title('Z Component');
xlabel('Time');
ylabel('Z');
legend('Ground Truth', 'Estimate');
grid on;
hold off;

% figure
% plot(Time(update_time<1) , update_time(update_time<1) , '.')
% title('update times')
% xlabel('Time')
% ylabel('updated')

% figure
% plot(Time , dtheta)
% xlabel('Time');
% ylabel('dtheta');
% grid on

% figure;
% subplot(3,1,1); % X component
% plot(Time , deul(1,:), 'r-','linewidth',1), hold on;
% 
% xlabel('Time');
% ylabel('angle 1');
% 
% grid on;
% hold off;
% 
% subplot(3,1,2); % Y component
% plot(Time , deul(2,:), 'r-','linewidth',1), hold on;
% xlabel('Time');
% ylabel('angle 2');
% grid on;
% hold off;
% 
% subplot(3,1,3); % Z component
% plot(Time , deul(3,:), 'r-','linewidth',1), hold on;
% xlabel('Time');
% ylabel('angle 3');
% grid on;
hold off;

figure;
subplot(3,1,1); % X component
plot(Time , drotv(1,:), 'r-','linewidth',1), hold on;

xlabel('Time');
ylabel('rotv 1');

grid on;
hold off;

subplot(3,1,2); % Y component
plot(Time , drotv(2,:), 'r-','linewidth',1), hold on;
xlabel('Time');
ylabel('rotv 2');
grid on;
hold off;

subplot(3,1,3); % Z component
plot(Time , drotv(3,:), 'r-','linewidth',1), hold on;
xlabel('Time');
ylabel('rotv 3');
grid on;
hold off;

% figure
% plot(Time , vecnorm(drotv), 'r-','linewidth',1)
% xlabel('Time');
% ylabel('rotv_norm');
% grid on;

figure
plot(Time , rad2deg(vecnorm(drotv)), 'r-','linewidth',1)
xlabel('Time');
ylabel('rotv_norm (degree)');
grid on;

figure
plot(update_time, update_state , '.')
title('update times')
xlabel('Time')
ylabel('updated')


% % Plotting X, Y, and Z components separately for Ground Truth
% figure;
% 
% subplot(3,1,1); % X component
% plot(PGrdtruth(1,:), 'g--','linewidth',1);
% title('X Component of Ground Truth');
% xlabel('Time');
% ylabel('X');
% grid on;
% 
% subplot(3,1,2); % Y component
% plot(PGrdtruth(2,:), 'g--','linewidth',1);
% title('Y Component of Ground Truth');
% xlabel('Time');
% ylabel('Y');
% grid on;
% 
% subplot(3,1,3); % Z component
% plot(PGrdtruth(3,:), 'g--','linewidth',1);
% title('Z Component of Ground Truth');
% xlabel('Time');
% ylabel('Z');
% grid on;

% set(gcf, 'Renderer', 'Painters');
% print('-depsc','E:\Dropbox (Personal)\Research Note\2-SE(3)\IEEE\3DTrajEx.eps')


% temp1 = sqrt(sum(Pestimate.^2,2));
% temp2 = sqrt(sum(Groundtruth(1:length(Pestimate),2:4).^2,2));
% RMSE = sqrt(mean((temp1-temp2).^2)); 

toc

function [IMURead,ViconRead,Groundtruth,CamRead] = dataRead(path)

% IMU
fileID      = fopen([path '\imu0\data.csv']);
csvRawIMU   = textscan(fileID, '%f,%f,%f,%f,%f,%f,%f', 'headerLines', 1);
fclose(fileID);
IMURead     = cell2mat(csvRawIMU); 
% IMURead   = csvread([path '\imu0\data.csv'],1,0);


% Vicon
fileID      = fopen([path '\vicon0\data.csv']);
csvRawVicon = textscan(fileID, '%f,%f,%f,%f,%f,%f,%f,%f', 'headerLines', 1);
fclose(fileID);
ViconRead   = cell2mat(csvRawVicon);  
% ViconRead = csvread([path '\vicon0\data.csv'],1,0);

% state_groundtruth
fileID      = fopen([path '\state_groundtruth_estimate0\data.csv']);
csvRawState = textscan(fileID, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f',...
                        'headerLines', 1);
fclose(fileID);
Groundtruth = cell2mat(csvRawState);  
% Groundtruth = csvread([path '\state_groundtruth_estimate0\data.csv'],1,0); 

fileID      = fopen([path '\cam0\data.csv']);
csvRawCam   = textscan(fileID, '%f,%s', 'headerLines', 1);
fclose(fileID);
CamRead     = [csvRawCam{1}];
% CamRead   = csvread([path '\cam0\data.csv'], '%d%d',1,1);


intTime     = intersect(IMURead(:,1),Groundtruth(:,1),'sorted');
timestart   = find(intTime(1,1)==IMURead(:,1));
timeend     = find(intTime(end,1)==IMURead(:,1));
IMURead     = IMURead(timestart:timeend,:);
timestart   = find(intTime(1,1)==Groundtruth(:,1));
timeend     = find(intTime(end,1)==Groundtruth(:,1));
Groundtruth = Groundtruth(timestart:timeend,:);

timestart   = find(CamRead(:,1)<Groundtruth(1,1), 1, 'last' );  %last max
timeend     = find(CamRead(:,1)>Groundtruth(end,1),1);          %first min
CamRead     = CamRead(timestart:timeend,:);

end

function [imageLeft,imageRight] = stereoRead(path,CamRead,k,stereoParams)
% read and rectify with stereoParams
imageLeft  = imread([path '\cam0\data\',num2str(CamRead(k),'%d'),'.png']);
imageRight = imread([path '\cam1\data\',num2str(CamRead(k),'%d'),'.png']);
[imageLeft,imageRight] = rectifyStereoImages(imageLeft,imageRight,stereoParams);
end

function [positionsB] = funPoints3D(pointsFeaturesL,pointsFeaturesR,stereoParams,TB2C0)
%     position on the frame of left cam 
positionCam0 = 0.001*triangulate(pointsFeaturesL,pointsFeaturesR,stereoParams);
positionCam0 = positionCam0';
%     positionCam0 = R'(positionsB-p)
R = TB2C0(1:3,1:3);
p = TB2C0(1:3,4);
positionsB = R*positionCam0 + p;    %R'*(positionCam0-p);%  
    
end

function [pointsKeyR,pointsKeyL,pointTracker] = funPointTracker(imageLeft,imageRight,numFeatures,pointTracker)

% feature detection on the right image of keyframe and set the points as
% pointTracker

points          = detectMinEigenFeatures(imageRight);
pointsKeyR      = points.Location;

release(pointTracker);
% Initialize the tracker with the initial point locations and the initial video frame.
initialize(pointTracker, pointsKeyR, imageRight);
% track features on the left image of keyframe
% [pointsKeyL, isFound] = step(pointTracker, imageLeft);% before R2016b
[pointsKeyL, isFound] = pointTracker(imageLeft);
% outlier removal
[pointsKeyL, isFound] = outlierRemoval(pointsKeyL,isFound);

pointsNewR = pointsKeyR(isFound,:);
pointsNewL = pointsKeyL(isFound,:);
 
% choose high socres munFeatures for efficiency
if size(pointsNewR, 1)>numFeatures
    randIndex  = randperm(size(pointsNewR, 1),numFeatures);
    pointsKeyR = pointsNewR(randIndex,:); 
    pointsKeyL = pointsNewL(randIndex,:); 
end

% reload tracker
setPoints(pointTracker,pointsKeyR); 
end

function [pointsNewL,pointsNewR,isFoundand] = stereoPointTrack(imageLeft,imageRight,pointTracker) 
% features tacking on new stereo images
% [pointsNewR, isFoundR]  = step(pointTracker, imageRight);   % before R2016b 
[pointsNewR, isFoundR]  = pointTracker(imageRight); 
% [pointsNewL, isFoundL]  = step(pointTracker, imageLeft);% before R2016b
[pointsNewL, isFoundL]  = pointTracker(imageLeft);


% outlier removal
[pointsNewR, isFoundR] = outlierRemoval(pointsNewR,isFoundR);
[pointsNewL, isFoundL] = outlierRemoval(pointsNewL,isFoundL);
% find features in both images
isFoundand = and(isFoundR,isFoundL);
        
end






 

function XhatInv = InvX(Xhat)
Rhat = Xhat(1:3,1:3);
vhat = Xhat(1:3,4);
phat = Xhat(1:3,5); 
XhatInv = [Rhat' -Rhat'*vhat -Rhat'*phat;zeros(2,3) eye(2)];

end

function [points,isFound]=outlierRemoval(points,isFound)
points2 = points(isFound);
pointsmean = mean(points2,1);
temp = points2-pointsmean;
dist = sqrt(diag(temp*temp'));
distmean = mean(dist);
diststd  = std(dist);
index = abs(dist-distmean)<=diststd;
points2= points2(index,:);
isFound = ismember(points(:,1),points2(:,1));
end


function [pointsI,pointsB,pointsNum] = pointsSelection(pointsI,pointsB)
% pointsNum = size(pointsI,2);
% pointsIC   = sum(pointsI,2)/pointsNum; % center of landmarks
% tempI = diag((pointsI - pointsIC)'*(pointsI - pointsIC));
% index = find(tempI<4);
% pointsI = pointsI(:,index);
% pointsB = pointsB(:,index);
% 
% pointsNum = size(pointsB,2);
% pointsBC   = sum(pointsB,2)/pointsNum; % center of landmarks 

% pointsBC  = mean(pointsB,2);
% % pointsBSd = std(pointsB,0,2);
% tempB =  pointsB - pointsBC;
% tempB = sqrt(diag(tempB'*tempB));
% meanB  = mean(tempB);
% stdB = std(tempB);% 
% index = find(abs(tempB-meanB)<max(stdB,0.3));
% pointsI = pointsI(:,index);
% pointsB = pointsB(:,index);

 


 
pointsIC   = mean(pointsI,2); % center of landmarks
pointsBC   = mean(pointsB,2); % center of landmarks
tempI = sqrt(diag((pointsI - pointsIC)'*(pointsI - pointsIC)));
tempB = sqrt(diag((pointsB - pointsBC)'*(pointsB - pointsBC)));
index = abs(tempI-tempB)<=0.01;
% error = abs(tempB - tempI);
% meane = mean(error);
% stde  = std(error);
% index   = find(abs(error-meane)<min(stde,0.009));
pointsI = pointsI(:,index);
pointsB = pointsB(:,index);

pointsNum = size(pointsI,2);
end

function wrappedAngles = wrapTo180(degrees)
    % Ensure input angles are within the valid range (-180 to 180 degrees)
    wrappedAngles = mod(degrees + 180, 360) - 180;
end




