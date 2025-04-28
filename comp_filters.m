clc; clear; close all;

% Load data into structures to handle variables with the same names
setpath = 2;
switch  setpath
    case 2 %exp1
        data_filter1 = load('data_QNUKF_20240818_194108_2_keep.mat');%mid 
        data_filter2 = load('data_OCEKF_20240818_225504_2_keep.mat');%V1_02
        data_filter3 = load('data_QNUKF_20240818_192143_2_keep.mat'); %mid
    case 3 %exp2
        data_filter1 = load('data_QNUKF_20240819_155240_3_keep.mat');%V1_03 
        data_filter2 = load('data_OCEKF_20240819_160834_3_keep.mat');%V1_03
        data_filter3 = load('data_QNUKF_20240819_153915_3_keep.mat'); %V1_03
    case 4 %exp3
        data_filter1 = load('data_EKF_20240819_171628_4_keep.mat');%V2_01 
        data_filter2 = load('data_OCEKF_20240819_170520_4_keep.mat');%V2_01
        data_filter3 = load('data_QNUKF_20240819_164751_4_keep.mat'); %V2_01 
end


% Define filter names for legends
filter_names = {'EKF', 'MSCKF', 'QUPF-VIN'};

% Ground truth data
PGrdtruth = data_filter1.Groundtruth(1:data_filter1.Tfinal, 2:4)';
p = data_filter1.Groundtruth(1:data_filter1.Tfinal , 2:4);
q = data_filter1.Groundtruth(1:data_filter1.Tfinal , 5:8);
v = data_filter1.Groundtruth(1:data_filter1.Tfinal , 9:11);

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

% Error calculations
[q_e1, p_e1, v_e1] = calculateErrors(q, qhat1, p, phat1, v, vhat1, data_filter1.Tfinal);
[q_e2, p_e2, v_e2] = calculateErrors(q, qhat2, p, phat2, v, vhat2, data_filter2.Tfinal);
[q_e3, p_e3, v_e3] = calculateErrors(q, qhat3, p, phat3, v, vhat3, data_filter3.Tfinal);

my_red = [1 , 0 , 0];
my_green = [0 , 0.8 , 0];
my_blue = [0 , 0 , 0.6];

% Plotting quaternion errors for all filters
figure('Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]); % Square figure
axisFontSize = 17; % Font size for axes labels

subplot(3, 1, 1);
customPlot(data_filter1.Time, vecnorm(q_e1'), 'Time [s]', '||r_{e,k}|| [rad.m]', 'plot', 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
hold on;
customPlot(data_filter2.Time, vecnorm(q_e2'), 'Time [s]', '||r_{e,k}|| [rad.m]', 'plot', 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
customPlot(data_filter3.Time, vecnorm(q_e3'), 'Time [s]', '||r_{e,k}|| [rad.m]', 'plot', 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
legend(filter_names);
% title('Quaternion Error');
ylim([min([vecnorm(q_e1'), vecnorm(q_e2'), vecnorm(q_e3')]) * 1.1 - 0.1, ...
      max([vecnorm(q_e1'), vecnorm(q_e2'), vecnorm(q_e3')]) * 1.1]);
grid on;
set(gca, 'FontSize', axisFontSize); % Set axis font size


% Plotting position errors for all filters
subplot(3, 1, 2);
customPlot(data_filter1.Time, vecnorm(p_e1'), 'Time [s]', '||p_{e,k}|| [m]', 'plot', 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
hold on;
customPlot(data_filter2.Time, vecnorm(p_e2'), 'Time [s]', '||p_{e,k}|| [m]', 'plot', 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
customPlot(data_filter3.Time, vecnorm(p_e3'), 'Time [s]', '||p_{e,k}|| [m]', 'plot', 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
legend(filter_names);
% title('Position Error');
ylim([min([vecnorm(p_e1'), vecnorm(p_e2'), vecnorm(p_e3')]) * 1.1- 2, ...
      max([vecnorm(p_e1'), vecnorm(p_e2'), vecnorm(p_e3')]) * 1.1]);
grid on;
set(gca, 'FontSize', axisFontSize); % Set axis font size

% Plotting velocity errors for all filters
subplot(3, 1, 3);
customPlot(data_filter1.Time, vecnorm(v_e1'), 'Time [s]', '||v_{e,k}|| [m/s]', 'plot', 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
hold on;
customPlot(data_filter2.Time, vecnorm(v_e2'), 'Time [s]', '||v_{e,k}|| [m/s]', 'plot', 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
customPlot(data_filter3.Time, vecnorm(v_e3'), 'Time [s]', '||v_{e,k}|| [m/s]', 'plot', 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
legend(filter_names);
% title('Velocity Error');
ylim([min([vecnorm(v_e1'), vecnorm(v_e2'), vecnorm(v_e3')]) * 1.1-2, ...
      max([vecnorm(v_e1'), vecnorm(v_e2'), vecnorm(v_e3')]) * 1.1]);
grid on;
set(gca, 'FontSize', axisFontSize); % Set axis font size
% Save the plots
saveas(gcf, 'error_compare_1.eps', 'epsc');
saveas(gcf, 'error_compare_1.fig');
saveas(gcf, 'error_compare_1.png');

% % Plotting comparison of errors between the second and third filters
% figure('Position', [100, 100, 800, 600]);
% 
% % Quaternion error comparison with colored positive/negative differences
% subplot(3, 1, 1);
% plotDifference(data_filter2.Time, vecnorm(q_e2') - vecnorm(q_e3'));
% title('Quaternion Error Difference (OC - QNUKF)');
% xlabel('Time [s]');
% ylabel('Error Difference');
% grid on;
% 
% % Position error comparison with colored positive/negative differences
% subplot(3, 1, 2);
% plotDifference(data_filter2.Time, vecnorm(p_e2') - vecnorm(p_e3'));
% title('Position Error Difference (OC - QNUKF)');
% xlabel('Time [s]');
% ylabel('Error Difference');
% grid on;
% 
% % Velocity error comparison with colored positive/negative differences
% subplot(3, 1, 3);
% plotDifference(data_filter2.Time, vecnorm(v_e2') - vecnorm(v_e3'));
% title('Velocity Error Difference (OC - QNUKF)');
% xlabel('Time [s]');
% ylabel('Error Difference');
% grid on;


% Calculate RMSE for all data points
rmse_all_1 = calculateRMSE(q_e1, p_e1, v_e1);
rmse_all_2 = calculateRMSE(q_e2, p_e2, v_e2);
rmse_all_3 = calculateRMSE(q_e3, p_e3, v_e3);

% Calculate RMSE for the last 20,000 data points
if size(q_e1 , 1)>20*200
    rmse_last_20000_1 = calculateRMSE(q_e1(end-(20*200-1):end, :), p_e1(end-(20*200-1):end, :), v_e1(end-(20*200-1):end, :));
    rmse_last_20000_2 = calculateRMSE(q_e2(end-(20*200-1):end, :), p_e2(end-(20*200-1):end, :), v_e2(end-(20*200-1):end, :));
    rmse_last_20000_3 = calculateRMSE(q_e3(end-(20*200-1):end, :), p_e3(end-(20*200-1):end, :), v_e3(end-(20*200-1):end, :));
else
    rmse_last_20000_1 = 0;
    rmse_last_20000_2 = 0;
    rmse_last_20000_3 = 0;
end

% Print RMSE values
fprintf('RMSE for all data points:\n');
fprintf('%s: %.6f\n', filter_names{1}, rmse_all_1);
fprintf('%s: %.6f\n', filter_names{2}, rmse_all_2);
fprintf('%s: %.6f\n', filter_names{3}, rmse_all_3);

fprintf('\nRMSE for the last 20 seconds data points:\n');
fprintf('%s: %.6f\n', filter_names{1}, rmse_last_20000_1);
fprintf('%s: %.6f\n', filter_names{2}, rmse_last_20000_2);
fprintf('%s: %.6f\n', filter_names{3}, rmse_last_20000_3);


% Function to calculate RMSE
function rmse = calculateRMSE(q_e, p_e, v_e)
    % Combine all errors into a single matrix
    errors = [q_e; p_e; v_e];
    
    % Calculate RMSE
    rmse = sqrt(mean(errors(:).^2));
end

% Function to calculate errors
function [q_e, p_e, v_e] = calculateErrors(q, qhat, p, phat, v, vhat, Tfinal)
    m_k = Tfinal; % Number of samples (should match the time steps)
    q_e = zeros(m_k, 3);
    p_e = zeros(size(p));
    v_e = zeros(size(v));
    
    for i = 1:Tfinal
        q_e(i, :) = qMq(q(i, :)', qhat(i, :)')';
        p_e(i, :) = p(i, :) - phat(i, :);
        v_e(i, :) = v(i, :) - vhat(i, :);
    end
    
    % Calculate RMSE for each component
    rmse_q = sqrt(sum(vecnorm(q_e').^2) / m_k);
    rmse_p = sqrt(sum(vecnorm(p_e').^2) / m_k);
    rmse_v = sqrt(sum(vecnorm(v_e').^2) / m_k);
    
    % Print the RMSE values
    fprintf('\nRMSE of components:\n');
    fprintf('Quaternion error (q_e): %.4f %.4f %.4f\n', rmse_q);
    fprintf('Position error (p_e): %.4f %.4f %.4f\n', rmse_p);
    fprintf('Velocity error (v_e): %.4f %.4f %.4f\n', rmse_v);
end

function plotDifference(Time, difference)
    % Create a colormap with two colors: green for positive, red for negative
    cmap = [0, 1, 0;  % Green
            1, 0, 0]; % Red

    % Create an array of colors based on the sign of the difference
    colorIndices = difference > 0; % Logical array: 1 for positive, 0 for negative

    % Map the color indices to the colormap
    colors = cmap(colorIndices + 1, :); % +1 because MATLAB indices start from 1

    % Use a scatter plot for efficiency
    scatter(Time, difference, [], colors, 'filled');

    % Set plot attributes
    hold on;
    plot(Time, difference, 'LineWidth', 1.5); % Optional: add a line to connect the points
    hold off;
end