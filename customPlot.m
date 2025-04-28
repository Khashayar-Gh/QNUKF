function customPlot(x, y, X_Label , Y_Label, plotType , varargin)
    % customPlot - Flexible plotting within a MATLAB figure.
    %
    % Syntax: customPlot(x, y, plotType, 'Name', Value, ...)
    %
    % Inputs:
    %   x - X data
    %   y - Y data
    %   plotType - Type of the plot ('plot', 'scatter', 'bar', etc.)
    %   varargin - Additional name-value pairs passed to the plotting function

    % Check if 'hold on' is active
    holdState = ishold;
    
    % Plot according to the specified type
    switch plotType
        case 'plot'
            plot(x, y, varargin{:});
        case 'scatter'
            scatter(x, y, varargin{:});
        case 'bar'  
            bar(x, y, varargin{:});
        otherwise
            error('Unsupported plot type.');
    end
    
    % Apply 'hold on' if it was active
    if holdState
        hold on;
    end
    
    % Set common properties for the plot
    
    set(gca, 'FontSize', 12); % Example: set font size for axis labels and ticks
    xlabel(X_Label);
    ylabel(Y_Label);
end
