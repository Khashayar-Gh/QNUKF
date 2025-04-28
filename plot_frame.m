function fig = plot_frame(q, p, qhat, phat, k, imgL, imgR, pointL, pointR, Time,rotv_e,p_e,v_e)
    fig = figure;
    fig.Position = [1, 1, 1920, 1080];
    t = tiledlayout(8*3, 6*3);  % Set the layout to 8 rows and 6 columns

    % Plot Stereo Left Image in the first four rows across the first three columns
    ax1 = nexttile(1, [3*3, 3*3]);  % Tile index 1, spanning 4 rows and 3 columns
    imshow(imgL, 'Parent', ax1);
    hold on;
    plot(pointL(:,1), pointL(:,2), 'c*', 'MarkerSize',10,'LineWidth', 1); % Plotting feature points as yellow circles
    title('Left Image with Features');

    % Plot Stereo Right Image in the first four rows across the next three columns
    ax2 = nexttile(3*3+1, [3*3, 3*3]);  % Tile index 4, spanning 4 rows and 3 columns
    imshow(imgR, 'Parent', ax2);
    hold on;
    plot(pointR(:,1), pointR(:,2), 'm+' , 'MarkerSize',10,'LineWidth', 1); % Plotting feature points as yellow circles
    title('Right Image with Features');

    % Span the existing plots over the remaining 4 rows and 6 columns
    ax3 = nexttile(18*9+1, [5*3, 4*3]);  % Tile index 19, spanning 4 rows and all 6 columns
    plot3(p(:,1), p(:,2), p(:,3), '--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    
    hold on;

    % Plotting the coordinate axes at selected points
    axisLength = 0.30; % Adjusted for visual clarity
    
%     q_ = qhat(k, :)'; % Column vector of quaternion
%     R = quat2rotm(q_');
%     origin = phat(k, :);
%     plot3([origin(1) origin(1) + R(1,1) * axisLength], [origin(2) origin(2) + R(1,2) * axisLength], [origin(3) origin(3) + R(1,3) * axisLength], 'r-', 'LineWidth', 2);
%     plot3([origin(1) origin(1) + R(2,1) * axisLength], [origin(2) origin(2) + R(2,2) * axisLength], [origin(3) origin(3) + R(2,3) * axisLength], 'g-', 'LineWidth', 2);
%     plot3([origin(1) origin(1) + R(3,1) * axisLength], [origin(2) origin(2) + R(3,2) * axisLength], [origin(3) origin(3) + R(3,3) * axisLength], 'b-', 'LineWidth', 2);

    q_ = q(k, :)'; % Column vector of quaternion
    R = quat2rotm(q_');
    origin = p(k, :);
    plot3([origin(1) origin(1) + R(1,1) * axisLength],...
        [origin(2) origin(2) + R(1,2) * axisLength],...
        [origin(3) origin(3) + R(1,3) * axisLength],...
        '--','LineWidth', 2,'color' , [0.6350 0.0780 0.1840]);
    plot3([origin(1) origin(1) + R(2,1) * axisLength],...
        [origin(2) origin(2) + R(2,2) * axisLength],...
        [origin(3) origin(3) + R(2,3) * axisLength],...
         '--','LineWidth', 2,'color' , [0.4660 0.6740 0.1880]);
    plot3([origin(1) origin(1) + R(3,1) * axisLength],...
        [origin(2) origin(2) + R(3,2) * axisLength],...
        [origin(3) origin(3) + R(3,3) * axisLength],...
        '--','LineWidth', 2,'color' , [0 0.4470 0.7410]);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on;
    title('Dataset Trajectory', 'Parent', ax3);
    ax3.FontSize = 12;

    x_min = min(p(:,1)); x_max = max(p(:,1));
    y_min = min(p(:,2)); y_max = max(p(:,2));
    z_min = min(p(:,3)); z_max = max(p(:,3));
    
    % Add a buffer for clarity
    buffer = 0.7; % Adjust buffer size as needed
    
    xlim([x_min-buffer, x_max+buffer]);
    ylim([y_min-buffer, y_max+buffer]);
    zlim([z_min-buffer, z_max+buffer]);

    nexttile(18*9+1+4*3 , [5,6]);
    y = vecnorm(rotv_e');
    plot(Time(1:k) , y(1:k) , 'LineWidth',2 , 'Color','blue')
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
    y =  y(1:k);
    y_min = min(y);
    y_max = max(y);
    ylim([1.1*y_min , 1.1*y_max])
    ax = gca;
    ax.FontSize = 12;
    ylabel('||r_{e,k}||')
    % Second small plot in the second row, second column
    nexttile(18*9+1+4*3+18*5 , [5,6]);
    y = vecnorm(p_e');
    plot(Time(1:k) , y(1:k) , 'LineWidth',2 , 'Color','blue')
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
    y = y(1:k);
    y_min = min(y);
    y_max = max(y);
    ylim([1.1*y_min , 1.1*y_max])
    ax = gca;
    ax.FontSize = 12;
    ylabel('||p_{e,k}||')
    
    % Third small plot in the second row, second column (below the previous one)
    
    nexttile(355 , [5,6]);
    y = vecnorm(v_e');
    plot(Time(1:k) , y(1:k) , 'LineWidth',2 , 'Color','blue')
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
    y = y(1:k);
    y_min = min(y);
    y_max = max(y);
    ylim([1.1*y_min , 1.1*y_max])
    ax = gca;
    ax.FontSize = 12;
    ylabel('||v_{e,k}||')
    xlabel('Time [s]')
    

    % Adjust the layout for visual clarity
    t.TileSpacing = 'compact';
    t.Padding = 'compact';
end
