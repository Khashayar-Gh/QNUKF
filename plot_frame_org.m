function fig = plot_frame_org(q, p, qhat1, qhat2, qhat3, phat1, phat2, ...
    phat3, k, imgL, imgR, pointL, pointR, Time, q_e1, q_e2, q_e3, p_e1, ...
    p_e2, p_e3, v_e1, v_e2, v_e3,filter_names)
    fig = figure('Visible', 'off');
    fig.Position = [1, 1, 1920, 1080];
    t = tiledlayout(8*3, 6*3);  % Set the layout to 8 rows and 6 columns

    % Plot Stereo Left Image in the first four rows across the first three columns
    ax1 = nexttile(1, [3*3, 3*3]);  % Tile index 1, spanning 4 rows and 3 columns
    imshow(imgL, 'Parent', ax1);
    hold on;
    plot(pointL(:,1), pointL(:,2), 'ro', 'MarkerSize',10,'LineWidth', 1); % Plotting feature points as yellow circles
    title('Left Image with Features');

    % Plot Stereo Right Image in the first four rows across the next three columns
    ax2 = nexttile(3*3+1, [3*3, 3*3]);  % Tile index 4, spanning 4 rows and 3 columns
    imshow(imgR, 'Parent', ax2);
    hold on;
    plot(pointR(:,1), pointR(:,2), 'bo' , 'MarkerSize',10,'LineWidth', 1); % Plotting feature points as yellow circles
    title('Right Image with Features');

    % Span the existing plots over the remaining 4 rows and 6 columns
    ax3 = nexttile(18*9+1, [5*3, 4*3]);  % Tile index 19, spanning 4 rows and all 6 columns
    plot3(p(:,1), p(:,2), p(:,3), '--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    
    hold on;

    % Plotting the coordinate axes at selected points for all three filters
    axisLength = 0.30; % Adjusted for visual clarity
    my_red = [1 , 0 , 0];
    my_green = [0 , 0.8 , 0];
    my_blue = [0 , 0 , 0.6];
    colors = {my_red, my_green, my_blue};
    phats = {phat1, phat2, phat3};
    qhats = {qhat1, qhat2, qhat3};
    for f = 1:3
        q_ = qhats{f}(k, :)'; % Column vector of quaternion
        R = quat2rotm(q_');
        origin = phats{f}(k, :);
        % Plot X, Y, Z axes for each orientation
        plot3([origin(1), origin(1) + R(1,1) * axisLength], [origin(2), origin(2) + R(2,1) * axisLength], [origin(3), origin(3) + R(3,1) * axisLength], 'color', colors{f}, 'LineWidth', 2);
        plot3([origin(1), origin(1) + R(1,2) * axisLength], [origin(2), origin(2) + R(2,2) * axisLength], [origin(3), origin(3) + R(3,2) * axisLength], 'color', colors{f}, 'LineWidth', 2);
        plot3([origin(1), origin(1) + R(1,3) * axisLength], [origin(2), origin(2) + R(2,3) * axisLength], [origin(3), origin(3) + R(3,3) * axisLength], 'color', colors{f}, 'LineWidth', 2);
    end

    % Plotting ground truth coordinate axes
    q_ = q(k, :)'; % Column vector of quaternion
    R = quat2rotm(q_');
    origin = p(k, :);
    plot3([origin(1) origin(1) + R(1,1) * axisLength], ...
        [origin(2) origin(2) + R(1,2) * axisLength], ...
        [origin(3) origin(3) + R(1,3) * axisLength], '-','LineWidth', ...
        2,'color' , [0.5 0.5 0.5]);
    plot3([origin(1) origin(1) + R(2,1) * axisLength], ...
        [origin(2) origin(2) + R(2,2) * axisLength], ...
        [origin(3) origin(3) + R(2,3) * axisLength], '-','LineWidth', ...
        2,'color' , [0.5 0.5 0.5]);
    plot3([origin(1) origin(1) + R(3,1) * axisLength], ...
        [origin(2) origin(2) + R(3,2) * axisLength], ...
        [origin(3) origin(3) + R(3,3) * axisLength], ...
        '-','LineWidth', 2,'color' , [0.5 0.5 0.5]);

    % Define a basic pinhole camera model
    cameraSize = 0.5;  % Adjust size of the camera model for clarity
    
    % Create camera vertices (a small pyramid-like structure for the pinhole camera)
    camVertices = cameraSize * [0 0 0; -0.5 -0.5 1; 0.5 -0.5 1; 0.5 0.5 1; -0.5 0.5 1];
    
    % Transform camera vertices based on ground truth orientation and position
    R_cam = quat2rotm(q(k, :));  % Get the rotation matrix from the quaternion
    camVerticesTransformed = (R_cam * camVertices')' + p(k, :);  % Apply rotation and translation
    
    % Define camera faces (connecting vertices to form a pyramid)
    camFaces = [1 2 3; 1 3 4; 1 4 5; 1 5 2];  % Only triangular faces for the camera
    
    % Define colors for each face (4 faces, so 4 colors)
    faceColors = [1 0 0;   % Red for one face
                  0 1 0;   % Green for another face
                  0 0 1;   % Blue for another face
                  1 1 0];  % Yellow for the final face
    
    % Plot the camera model using patch
    for i = 1:size(camFaces, 1)
        patch('Vertices', camVerticesTransformed, 'Faces', camFaces(i, :), ...
              'FaceColor', faceColors(i, :), 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    title('Dataset Trajectory', 'Parent', ax3);
    ax3.FontSize = 12;

%     x_min = min([min(p(:,1)), min(phat1(:,1)), min(phat2(:,1)), min(phat3(:,1))]);
%     x_max = max([max(p(:,1)), max(phat1(:,1)), max(phat2(:,1)), max(phat3(:,1))]);
%     y_min = min([min(p(:,2)), min(phat1(:,2)), min(phat2(:,2)), min(phat3(:,2))]);
%     y_max = max([max(p(:,2)), max(phat1(:,2)), max(phat2(:,2)), max(phat3(:,2))]);
%     z_min = min([min(p(:,3)), min(phat1(:,3)), min(phat2(:,3)), min(phat3(:,3))]);
%     z_max = max([max(p(:,3)), max(phat1(:,3)), max(phat2(:,3)), max(phat3(:,3))]);
    
    x_min = min(p(:,1));
    x_max = max(p(:,1));
    y_min = min(p(:,2));
    y_max = max(p(:,2));
    z_min = min(p(:,3));
    z_max = max(p(:,3));
    % Add a buffer for clarity
    buffer = 0.5; % Adjust buffer size as needed
    
    xlim([x_min-buffer, x_max+buffer]);
    ylim([y_min-buffer, y_max+buffer]);
    zlim([z_min-buffer, z_max+buffer]);
    grid on;
    nexttile(18*9+1+4*3 , [5,6]);
    plot(Time, vecnorm(q_e1'), 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
    hold on;
    plot(Time, vecnorm(q_e2'), 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
    plot(Time, vecnorm(q_e3'), 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
    legend(filter_names , Location="northeast");
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
%     y =  y(1:k);
%     y_min = min(y);
%     y_max = max(y);
    ylim([min([vecnorm(q_e1'), vecnorm(q_e2'), vecnorm(q_e3')]) * 1.1 - 0.1, ...
      max([vecnorm(q_e1'), vecnorm(q_e2'), vecnorm(q_e3')]) * 1.1]);
    ax = gca;
    ax.FontSize = 12;
    ylabel('||r_{e,k}||')

    % Second small plot in the second row, second column
    nexttile(18*9+1+4*3+18*5 , [5,6]);
    y = vecnorm(p_e1');
    plot(Time, vecnorm(p_e1'), 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
    hold on;
    plot(Time, vecnorm(p_e2'), 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
    plot(Time, vecnorm(p_e3'), 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
    legend(filter_names , Location="northeast");
    % title('Position Error');
    ylim([min([vecnorm(p_e1'), vecnorm(p_e2'), vecnorm(p_e3')]) * 1.1- 2, ...
          max([vecnorm(p_e1'), vecnorm(p_e2'), vecnorm(p_e3')]) * 1.1]);
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
    y = y(1:k);
    y_min = min(y);
    y_max = max(y);
%     ylim([1.1*y_min , 1.1*y_max])
    ax = gca;
    ax.FontSize = 12;
    ylabel('||p_{e,k}||')
    
    % Third small plot in the second row, second column (below the previous one)
    
    nexttile(355 , [5,6]);
    y = vecnorm(v_e1');
    
    plot(Time, vecnorm(v_e1'), 'Color', my_red, 'LineWidth', 6, 'LineStyle' , '-');
    hold on;
    plot(Time, vecnorm(v_e2'), 'Color', my_green, 'LineWidth', 5 , 'LineStyle' , '-');
    plot(Time, vecnorm(v_e3'), 'Color', my_blue, 'LineWidth', 2.5, 'LineStyle' , '-.');
    legend(filter_names, Location="northeast");
    % title('Velocity Error');
    ylim([min([vecnorm(v_e1'), vecnorm(v_e2'), vecnorm(v_e3')]) * 1.1-2, ...
          max([vecnorm(v_e1'), vecnorm(v_e2'), vecnorm(v_e3')]) * 1.1]);
    % title('Small Plot 1');
    xlim([Time(1) , Time(k)])
    y = y(1:k);
    y_min = min(y);
    y_max = max(y);
%     ylim([1.1*y_min , 1.1*y_max])
    ax = gca;
    ax.FontSize = 12;
    ylabel('||v_{e,k}||')
    xlabel('Time [s]')
    

    % Adjust the layout for visual clarity
    t.TileSpacing = 'compact';
    t.Padding = 'compact';
end
