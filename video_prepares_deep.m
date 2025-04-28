clc;clear;close all;
% load('data_mid_VeryGood.mat'); 
load('data_QNUKF_20240521_152058.mat')
% load('data_QNUKF_20240705_100305.mat');
%load('data_QNPF_20240906_100136_2.mat')

Groundtruth = readmatrix("C:\Users\khash\OneDrive\Desktop\Deep_true_state_UKF.csv");
Xhat_log = readmatrix("C:\Users\khash\OneDrive\Desktop\Deep_estimated_state_UKF.csv");
Tfinal = size(Groundtruth,1);
Time = readmatrix("C:\Users\khash\OneDrive\Desktop\Deep_time.csv");

% p = Groundtruth(1:Tfinal , 2:4);
% q = Groundtruth(1:Tfinal , 5:8);
% v = Groundtruth(1:Tfinal , 9:11);

q = Groundtruth(1:Tfinal , 1:4);
p = Groundtruth(1:Tfinal , 5:7);
v = Groundtruth(1:Tfinal , 8:10);

qhat = Xhat_log(1:4 , :)';
phat = Xhat_log(5:7 , :)';
vhat = Xhat_log(8:10 , :)';

q_e = zeros(size(qhat));
axang= zeros(size(qhat));
axanghat= zeros(size(qhat));
eul = zeros(Tfinal , 3);
rotv_e = zeros(Tfinal , 3);
rotv = zeros(Tfinal , 3);
rotvhat = zeros(Tfinal , 3);
eulhat = zeros(size(eul));
p_e = p - phat;
v_e = v - vhat;
for i=1:Tfinal
    q_e(i , :) = qMq_q(q(i , :)' , qhat(i , :)')';
    rotv_e(i , :)= qMq(q(i , :)' , qhat(i , :)')';
    
%     if qhat(i , 1)<0
%        qhat(i , 1) = qhat(i , 1)*-1;
%     end
%     if q(i , 1)<0
%        q(i , 1) = q(i , 1)*-1;
%     end
    q(i , :) = rotm2quat(quat2rotm(q(i , :)));
    qhat(i , :) = rotm2quat(quat2rotm(qhat(i , :)));
    eul(i , :) = quat2eul(q(i , :) , 'XYZ');
    eulhat(i , :) = quat2eul(qhat(i , :), 'XYZ');
    axang(i , :) = quat2axang(q(i , :));
    axanghat(i , :) = quat2axang(qhat(i , :));
    axang(i , 4) = rad2deg(axang(i , 4));
    axanghat(i , 4) = rad2deg(axanghat(i , 4));
    rotvhat(i , :) = axanghat(i , 4)*axanghat(i , 3);
    rotv(i , :) = axang(i , 4)*axang(i , 3);
end

videoFile = 'tracking_video_upf.avi';
writerObj = VideoWriter(videoFile, 'Motion JPEG AVI');
% writerObj.Quality = 95;
writerObj.FrameRate = 20;  % Adjust according to your data rate
open(writerObj);


numFeatures = 60;
index       = 1;
[imageLeft,imageRight]                       = stereoRead(path,CamRead,index,stereoParams);
% pointTracker = vision.PointTracker;
pointTracker = vision.PointTracker('BlockSize',[9 9],'MaxBidirectionalError',1); %Reduce the blocksize can improve the accuracy
k=1;
[pointsKeyR,pointsKeyL,pointTracker] = funPointTracker(imageLeft,imageRight,numFeatures,pointTracker);
p_ = p;
% fig = plot_frame(q, p_, qhat, phat, k, imageLeft, imageRight, pointsKeyL, pointsKeyR,Time,rotv_e,p_e,v_e);
% frame = getframe(fig);
% writeVideo(writerObj, frame);
close all; % Close the figure to free memory

for k=1:1:Tfinal
    if mod(floor(100*k/Tfinal),5)==0
    clc
    disp(['Complete  ' num2str(floor(100*k/Tfinal)) '%'])
    end
    % errors
    p       = Groundtruth(k,2:4)';
    R       = quat2rotm(Groundtruth(k,5:8));
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
%             pointsB = pointsB + z_std*randn(size(pointsB));
%             [state, params] = update(state, params,...
%                 pointsB + z_std*randn(size(pointsB)) ,...
%                 pointsNum , pointsI); 

            p= Groundtruth(k,2:4)';
            R_= quat2rotm(Groundtruth(k,5:8));
        
        end %endif
        
        % Creat new Keyframe if pointsNum is less than 6     
        minFeatures = 6;
         %size(find(isFoundand==1), 1) < minFeatures       
        [pointsKeyR,pointsKeyL,pointTracker] = funPointTracker(imageLeft,imageRight,20,pointTracker);                 
%         pkey      = Groundtruth(k,2:4)';
%         Rkey      = quat2rotm(Groundtruth(k,5:8));
%         points3D  = funPoints3D(pointsKeyL,pointsKeyR,stereoParams,TB2C0);
%         pointsKey = Rkey*points3D+pkey;  %inertial frame landmarks 
        keytime(index) = 1;
        fig = plot_frame(q, p_, qhat, phat, k+1, imageLeft, imageRight, pointsKeyL, pointsKeyR,Time,rotv_e,p_e,v_e);
        frame = getframe(fig);
        writeVideo(writerObj, frame);

         

         
         close all; % Close the figure to free memory
        
         
    end %endif
    
    
    Xhat = state.x;
end %endfor
close(writerObj);



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

