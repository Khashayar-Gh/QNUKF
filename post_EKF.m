clc;clear;close all;
% load('data_mid_VeryGood.mat')
% load('data_newbest_12052024_1107.mat');
% load('data_newgood_repeatable.mat');
% load('data_QNUKF_20240705_100305.mat')
load('data_QNUKF_20240806_144938_1.mat')
% load('data_QNUKF_20240521_152058.mat')
PGrdtruth = Groundtruth(1:Tfinal,2:4)';

p = Groundtruth(1:Tfinal , 2:4);
q = Groundtruth(1:Tfinal , 5:8);
v = Groundtruth(1:Tfinal , 9:11);

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

figure('Position',[999.4000  438.6000  541.6000  336.0000])
subplot(4 , 1 , 1)

% plot(Time , q(: , 1))
% hold on
% plot(Time , qhat(: , 1))
plot(Time , q_e(: , 1))
title('quat error')

yline(1)


subplot(4 , 1 , 2)
plot(Time , q_e(: , 2))
% hold on
% plot(Time , qhat(: , 1))
yline(0)


subplot(4 , 1 , 3)
plot(Time , q_e(: , 3))
yline(0)


subplot(4 , 1 , 4)
plot(Time , q_e(: , 4))
yline(0)


figure('Position',1.0e+03*[1.0074    0.0546    0.5184    0.3032])

subplot(4 , 1 , 1)

plot(Time , q(: , 1))
title('quat')

hold on
plot(Time , qhat(: , 1))
legend('Ground Truth' , 'Estimated')
yline(1)



subplot(4 , 1 , 2)
plot(Time , q(: , 2))
hold on
plot(Time , qhat(: , 2))
yline(0)


subplot(4 , 1 , 3)
plot(Time , q(: , 3))
hold on
plot(Time , qhat(: , 3))
yline(0)


subplot(4 , 1 , 4)
plot(Time , q(: , 4))
hold on
plot(Time , qhat(: , 4))
yline(0)

figure("Position",[1.0000   55.4000  560.0000  248.8000])

subplot(3 , 1 , 1)
plot(Time , eul(: , 1))
title('eul')

hold on
plot(Time , eulhat(: , 1))
grid on
legend('Ground Truth' , 'Estimated')

subplot(3 , 1 , 2)
plot(Time , eul(: , 2))
hold on
plot(Time , eulhat(: , 2))
grid on

subplot(3 , 1 , 3)
plot(Time , eul(: , 3))
hold on
plot(Time , eulhat(: , 3))
grid on

figure('Position',[7.4000  351.4000  560.0000  420.0000])

subplot(4 , 1 , 1)
plot(Time , axang(: , 1))
hold on
plot(Time , axanghat(: , 1))
legend('Ground Truth' , 'Estimated')
grid on



subplot(4 , 1 , 2)

plot(Time , axang(: , 2))
title('ax-ang')

hold on
plot(Time , axanghat(: , 2))
grid on


subplot(4 , 1 , 3)
plot(Time , axang(: , 3))
hold on
plot(Time , axanghat(: , 3))
grid on


subplot(4 , 1 , 4)
plot(Time , axang(: , 4))
hold on
plot(Time , axanghat(: , 4))
grid on

figure('Position',[491.4000   41.8000  391.2000  740.8000])
subplot(3 , 1 , 1)
customPlot(Time, rotv_e(:, 1), 'Time [s]', 'r_{e1 , k} [rad.m]', 'plot' ,'r', 'linewidth' , 2);
y_min = min(rotv_e(:, 1));
y_max = max(rotv_e(:, 1));
xlim([Time(1) , Time(end)])
ylim([1.1*y_min , 1.1*y_max])
% title('rotv error')
grid on

subplot(3 , 1 , 2)
customPlot(Time, rotv_e(:, 2), 'Time [s]', 'r_{e2 , k} [rad.m]', 'plot','r', 'linewidth' , 2);
y_min = min(rotv_e(:, 2));
y_max = max(rotv_e(:, 2));
xlim([Time(1) , Time(end)])
ylim([1.1*y_min , 1.1*y_max])
grid on

subplot(3 , 1 , 3)
customPlot(Time, rotv_e(:, 3), 'Time [s]', 'r_{e3 , k} [rad.m]', 'plot','r', 'linewidth' , 2);
y_min = min(rotv_e(:, 3));
y_max = max(rotv_e(:, 3));
xlim([Time(1) , Time(end)])
ylim([1.1*y_min , 1.1*y_max])
grid on


figure('Position',[486.6000   62.6000  560.0000  266.4000])

subplot(3 , 1 , 1)
plot(Time , rotv(: , 1))
title('rotv')

hold on
plot(Time , rotvhat(: , 1))
grid on
legend('Ground Truth' , 'Estimated')

subplot(3 , 1 , 2)
plot(Time , rotv(: , 2))
hold on
plot(Time , rotvhat(: , 2))
grid on

subplot(3 , 1 , 3)
plot(Time , rotv(: , 3))
hold on
plot(Time , rotvhat(: , 3))
grid on

%% 3D

% Calculate the estimation error at each data point


% Create a tiled layout with 2 rows and 2 columns
fig = figure;
fig.Position=[1          41        1920         963];
t = tiledlayout(3, 4);

% Span the first tile over 2 rows and 1 column
nexttile(1, [3, 3]);
errors = vecnorm((PGrdtruth - Pestimate).^2);

% Determine points where to plot the axes (e.g., uniformly spaced)
% Define the number of points where to plot the axes
numPoints = 8;

% Ensure indices always include the first and last data points
if numPoints > 2
    indices = round(linspace(1, size(PGrdtruth, 2), numPoints));
else
    % If numPoints is 2 or less, directly use the first and last indices
    indices = [1, size(PGrdtruth, 2)];
end

% Avoid duplicates in case of very small arrays
indices = unique(indices);
% indices = [1        4093        8184       12276       16367]
% indices = [1        4093        8184       12276       ];



% Increase the axis length for better visualization
axisLength = 0.30; % Length of the coordinate axes, adjusted for visual clarity

% Plot the ground truth trajectory and color it based on error magnitude
% figureHandle = figure;
% set(gcf, 'Position', get(0, 'Screensize'));
% scatter3(PGrdtruth(1,:), PGrdtruth(2,:), PGrdtruth(3,:), 6, errors, 'filled',...
%     'MarkerEdgeAlpha',0.2 , 'MarkerFaceAlpha',0.2);
plot3(PGrdtruth(1,:), PGrdtruth(2,:), PGrdtruth(3,:) , '.','LineWidth',1 , 'Color','black')
hold on;

% Plotting the coordinate axes at selected points
for i = indices
    % Extract the quaternion [qw, qx, qy, qz]
    q_ = q(i , :)'; % Assuming 'quaternions' variable is defined correctly
    % Convert quaternion to a rotation matrix
    R = quat2rotm(q_');

    % Origin of the local coordinate system
    origin = PGrdtruth(:, i);
    
    % X-axis
    endX = origin + R(:,1) * axisLength;
    plot3([origin(1) endX(1)], [origin(2) endX(2)], [origin(3) endX(3)], 'r--', 'LineWidth', 2);
    
    % Y-axis
    endY = origin + R(:,2) * axisLength;
    plot3([origin(1) endY(1)], [origin(2) endY(2)], [origin(3) endY(3)], 'g--', 'LineWidth', 2);
    
    % Z-axis
    endZ = origin + R(:,3) * axisLength;
    plot3([origin(1) endZ(1)], [origin(2) endZ(2)], [origin(3) endZ(3)], 'b--', 'LineWidth', 2);
end

% Setting the main plot axes line width
ax = gca; % Get current axes
% ax.LineWidth = 2; % Set main axes frame to be thicker
ax.FontSize = 12;
% Enhance visualization
% colormap(summer); % Use gray scale colormap
% colorbarHandle = colorbar; % Display a color bar indicating the scale of errors
% ylabel(colorbarHandle, 'p_e (m)'); % Set label for the colorbar with units
% clim([min(errors), max(errors)]); % Set color limits based on error range

% Labels and titles for clarity
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;

% Improve rendering quality for saving images
set(gcf, 'Renderer', 'Painters');
title('Dataset Trajectory');


% Add three smaller plots to the right
% First small plot in the first row, second column
nexttile(4);
y = vecnorm(rotv_e');
plot(Time , y , 'LineWidth',2 , 'Color','red')
% title('Small Plot 1');
xlim([Time(1) , Time(end)])
y_min = min(y);
y_max = max(y);
ylim([1.1*y_min , 1.1*y_max])
ax = gca;
ax.FontSize = 12;
ylabel('||r_{e,k}|| [rad.m]')
% Second small plot in the second row, second column
nexttile(8);
y = vecnorm(p_e');
plot(Time , y , 'LineWidth',2 , 'Color','red')
% title('Small Plot 1');
xlim([Time(1) , Time(end)])
y_min = min(y);
y_max = max(y);
ylim([1.1*y_min , 1.1*y_max])
ax = gca;
ax.FontSize = 12;
ylabel('||p_{e,k}|| [m]')

% Third small plot in the second row, second column (below the previous one)

nexttile(12);
y = vecnorm(v_e');
plot(Time , y , 'LineWidth',2 , 'Color','red')
% title('Small Plot 1');
xlim([Time(1) , Time(end)])
y_min = min(y);
y_max = max(y);
ylim([1.1*y_min , 1.1*y_max])
ax = gca;
ax.FontSize = 12;
ylabel('||v_{e,k}|| [m/s]')
xlabel('Time [s]')

% Adjust the layout
t.TileSpacing = 'compact';
t.Padding = 'compact';

% Enhance visibility


% Set titles for small plots

print('-depsc', 'Summary.eps'); % Save the figure as a high-quality EPS file

% figure
% subplot(3 , 3 , 1)


%%
figure('Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]); % Square figure

% Font sizes
axisFontSize = 17; % Font size for axes labels
titleFontSize = 14; % Font size for titles

% Create a tiled layout
t = tiledlayout(3, 3);
t.TileSpacing = 'compact';
t.Padding = 'compact';

% --- Plotting rotv_e components ---
for i = 1:3
    ax = nexttile(3 * (i - 1) + 1);
    if i == 3 % Only add xlabel for the last row
        customPlot(Time, rotv_e(:, i), 'Time [s]', ['r_{e' num2str(i) ', k} [rad.m]'], 'plot', 'k', 'linewidth', 3);
        xlabel('Time [s]', 'FontSize', axisFontSize);
    else
        customPlot(Time, rotv_e(:, i), '', ['r_{e' num2str(i) ', k} [rad.m]'], 'plot', 'k', 'linewidth', 3);
    end
    ylabel(['r_{e' num2str(i) ', k} [rad.m]'], 'FontSize', axisFontSize);
    y_min = min(rotv_e(:, i));
    y_max = max(rotv_e(:, i));
    xlim([Time(1), Time(end)]);
    ylim([1.1*y_min, 1.1*y_max]);
    grid on;
    set(ax, 'FontSize', axisFontSize); % Set axis font size
end

% --- Plotting p_e components ---
for i = 1:3
    ax = nexttile(3 * (i - 1) + 2);
    if i == 3 % Only add xlabel for the last row
        customPlot(Time, p_e(:, i), 'Time [s]', ['p_{e' num2str(i) ', k} [m]'], 'plot', 'k', 'linewidth', 3);
        xlabel('Time [s]', 'FontSize', axisFontSize);
    else
        customPlot(Time, p_e(:, i), '', ['p_{e' num2str(i) ', k} [m]'], 'plot', 'k', 'linewidth', 3);
    end
    ylabel(['p_{e' num2str(i) ', k} [m]'], 'FontSize', axisFontSize);
    y_min = min(p_e(:, i));
    y_max = max(p_e(:, i));
    xlim([Time(1), Time(end)]);
    ylim([1.1*y_min, 1.1*y_max]);
    grid on;
    set(ax, 'FontSize', axisFontSize); % Set axis font size
end

% --- Plotting v_e components ---
for i = 1:3
    ax = nexttile(3 * (i - 1) + 3);
    if i == 3 % Only add xlabel for the last row
        customPlot(Time, v_e(:, i), 'Time [s]', ['v_{e' num2str(i) ', k} [m/s]'], 'plot', 'k', 'linewidth', 3);
        xlabel('Time [s]', 'FontSize', axisFontSize);
    else
        customPlot(Time, v_e(:, i), '', ['v_{e' num2str(i) ', k} [m/s]'], 'plot', 'k', 'linewidth', 3);
    end
    ylabel(['v_{e' num2str(i) ', k} [m/s]'], 'FontSize', axisFontSize);
    y_min = min(v_e(:, i));
    y_max = max(v_e(:, i));
    xlim([Time(1), Time(end)]);
    ylim([1.1*y_min, 1.1*y_max]);
    grid on;
    set(ax, 'FontSize', axisFontSize); % Set axis font size
end


