function fig = createCustomFigure(width, height)
    % createCustomFigure - Creates a figure with custom dimensions.
    %
    % Syntax: fig = createCustomFigure(width, height)
    %
    % Inputs:
    %   width - Width of the figure in inches
    %   height - Height of the figure in inches

    if nargin < 2
        % Default size for double-column conference paper (example sizes, adjust as needed)
        width = 4*.8*.6; % inches
        height = 6*.6; % inches
    end
    
%     fig = figure;
%     set(fig, 'Units', 'inches', 'Position', [1, 1, width, height]);
    fig = figure('Position',1e0*[1.8000   41.8000  438.4000*2  740.8000]);
end
