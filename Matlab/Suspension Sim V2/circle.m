%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method circle
% Purpose: Creates a drawing of the rover as the plot. Takes into account
%          the current coordinates and orientation of each link.
% Parameters:
%       obj -- the object to be drawn.
%       c -- the configuration of the object to be drawn.
%       i -- the iteration of the object to be drawn.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = circle(x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit);
end