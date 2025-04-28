clc; clear; close all;
% Load data for three filters
data_filter1 = load('data_QNUKF_20240819_155240_3.mat'); %V1_03 
data_filter2 = load('data_OCEKF_20240819_160834_3.mat'); %V1_03
data_filter3 = load('data_QNUKF_20240819_153915_3.mat'); %V1_03

% Define filter names for legends
filter_names = {'EKF', 'MSCKF', 'QUPF-VIN'};
Tfinal = data_filter1.Tfinal;
% Ground truth data
p = data_filter1.Groundtruth(1:Tfinal , 2:4);
q = data_filter1.Groundtruth(1:Tfinal , 5:8);
v = data_filter1.Groundtruth(1:Tfinal , 9:11);

% Estimate data from three filters
qhat1 = data_filter1.Xhat_log(1:4 , :)';
phat1 = data_filter1.Xhat_log(5:7 , :)';
vhat1 = data_filter1.Xhat_log(8:10 , :)';

qhat2 = data_filter2.Xhat_log(1:4 , :)';
phat2 = data_filter2.Xhat_log(5:7 , :)';
vhat2 = data_filter2.Xhat_log(8:10 , :)';

qhat3 = data_filter3.Xhat_log(1:4 , :)';
phat3 = data_filter3.Xhat_log(5:7 , :)';
vhat3 = data_filter3.Xhat_log(8:10 , :)';

% Initialize error matrices
q_e1 = zeros(size(qhat1, 1) , 3);
p_e1 = p - phat1;
v_e1 = v - vhat1;

q_e2 = zeros(size(qhat2, 1) , 3);
p_e2 = p - phat2;
v_e2 = v - vhat2;

q_e3 = zeros(size(qhat3 , 1) , 3);
p_e3 = p - phat3;
v_e3 = v - vhat3;

% Calculate quaternion errors for all filters
for i = 1:Tfinal
    q_e1(i, :) = qMq(q(i, :)', qhat1(i, :)')';
    q_e2(i, :) = qMq(q(i, :)', qhat2(i, :)')';
    q_e3(i, :) = qMq(q(i, :)', qhat3(i, :)')';
end

% Video file setup
videoFile = 'tracking_video_comparison3';
writerObj = VideoWriter(videoFile, 'MPEG-4');
writerObj.FrameRate = 20;
open(writerObj);

numFeatures = 60;
index = 1;
[imageLeft, imageRight] = stereoRead(data_filter1.path, data_filter1.CamRead, index, data_filter1.stereoParams);
pointTracker = vision.PointTracker('BlockSize', [9 9], 'MaxBidirectionalError', 1);
[pointsKeyR, pointsKeyL, pointTracker] = funPointTracker(imageLeft, imageRight, numFeatures, pointTracker);
p_ = p;

for k = 1:Tfinal
    if mod(k, 20) == 0  % Reduce frequency of console output
        clc
        disp(['Complete  ' num2str(floor(100 * k / Tfinal)) '%']);
    end
    
    % Errors
    p = data_filter1.Groundtruth(k, 2:4)';
    R = quat2rotm(data_filter1.Groundtruth(k, 5:8));
    
    if data_filter1.TimerIMU(k + 1) >= data_filter1.TimerCam(index + 1, 1)
        index = index + 1;
        [imageLeft, imageRight] = stereoRead(data_filter1.path, data_filter1.CamRead, index, data_filter1.stereoParams);
        [pointsNewL, pointsNewR, isFoundand] = stereoPointTrack(imageLeft, imageRight, pointTracker);
        
        % Tracked features locations
        pointsNewR = pointsNewR(isFoundand, :);
        pointsNewL = pointsNewL(isFoundand, :);
        
        % Create new keyframe if pointsNum is less than 6
        minFeatures = 6;
        [pointsKeyR, pointsKeyL, pointTracker] = funPointTracker(imageLeft, imageRight, 20, pointTracker);
        keytime(index) = 1;
        fig = plot_frame_org(q, p_, qhat1, qhat2, qhat3, phat1, phat2, ...
            phat3, k + 1, imageLeft, imageRight, pointsKeyL, pointsKeyR, ...
            data_filter1.Time, q_e1, q_e2, q_e3, p_e1, p_e2, p_e3, v_e1, ...
            v_e2, v_e3 , filter_names);
        frame = getframe(fig);
        writeVideo(writerObj, frame);
        close all;
    end
end
close(writerObj);

% % Calculate RMSE for the last 20,000 data points
% if size(q_e1, 1) > 20 * 200
%     rmse_last_20000_1 = calculateRMSE(q_e1(end-(20*200-1):end, :), p_e1(end-(20*200-1):end, :), v_e1(end-(20*200-1):end, :));
%     rmse_last_20000_2 = calculateRMSE(q_e2(end-(20*200-1):end, :), p_e2(end-(20*200-1):end, :), v_e2(end-(20*200-1):end, :));
%     rmse_last_20000_3 = calculateRMSE(q_e3(end-(20*200-1):end, :), p_e3(end-(20*200-1):end, :), v_e3(end-(20*200-1):end, :));
% else
%     rmse_last_20000_1 = 0;
%     rmse_last_20000_2 = 0;
%     rmse_last_20000_3 = 0;
% end
% 
% % Print RMSE values
% fprintf('RMSE for all data points:\n');
% fprintf('%s: %.6f\n', filter_names{1}, rmse_all_1);
% fprintf('%s: %.6f\n', filter_names{2}, rmse_all_2);
% fprintf('%s: %.6f\n', filter_names{3}, rmse_all_3);
% 
% fprintf('\nRMSE for the last 20 seconds data points:\n');
% fprintf('%s: %.6f\n', filter_names{1}, rmse_last_20000_1);
% fprintf('%s: %.6f\n', filter_names{2}, rmse_last_20000_2);
% fprintf('%s: %.6f\n', filter_names{3}, rmse_last_20000_3);

% Function to calculate RMSE
function rmse = calculateRMSE(q_e, p_e, v_e)
    % Combine all errors into a single matrix
    errors = [q_e; p_e; v_e];
    % Calculate RMSE
    rmse = sqrt(mean(errors(:).^2));
end

% Function to calculate quaternion, position, and velocity errors
function [q_e, p_e, v_e] = calculateErrors(q, qhat, p, phat, v, vhat, Tfinal)
    q_e = zeros(size(qhat, 1), 3);
    p_e = p - phat;
    v_e = v - vhat;
    for i = 1:Tfinal
        q_e(i, :) = qMq_q(q(i, :)', qhat(i, :)')';
    end
end


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
