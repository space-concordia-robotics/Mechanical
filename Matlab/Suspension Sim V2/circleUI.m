%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method circleUI
% Purpose: Plot a circle of provided dimensions on Matlab App Designer.
% Parameters:
%       app -- the app where the circle will be plotted. 
%       x -- x coordinate of the center of the circle.
%       x -- y coordinate of the center of the circle.
%       r -- radius of the circle.
% Returns: The plot object corresponding to the circle.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = circleUI(app,x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(app.UIAxes, xunit, yunit);
end