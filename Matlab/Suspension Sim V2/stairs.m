%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Stairs class
% Author: Maxim Kaller
% Purpose: Represents a stair terrain on which the rover will drive. See
%          Stair.PNG for information on the meaning of each property.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef stairs < road
    properties
        tread
        riser
        slope
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method stairs (constructor)
% Purpose: Defines the class properties based on the inputs. It also
%          automatically calculates the gravity value that is used. See
%          Stair.PNG and the definitions in main.m for info on the terms
%          used.
% Parameters:
%       t -- The tread of the stairs.
%       r -- The riser of the stairs.
%       grav -- Gravitational constant used for simulations of the stair.
% Returns: The object which is being initialized.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = stairs(t, r, grav)
            obj.g = grav;
            obj.tread = t;
            obj.riser = r;
            if (obj.tread ~= 0)
                obj.slope = obj.riser / obj.tread;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method isOnStep
% Purpose: Determines whether the specifies input coordinate is on a step
%          in the specified stair.
% Parameters: 
%       obj -- the stair object that will be compared to the provided
%              x coordinate.
%       x -- the x coordinate that will be compared to the stair's steps.
% Returns: 
%       1 -- if the x coordinate coincides with one of the stair's steps.
%       0 -- if the x coordinate does not coincide with one of the stair's
%            steps.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function output = isOnStep(obj, x)
            if (mod(x,obj.tread) == 0) && (x ~= 0)
                output = 1;
            else
                output = 0;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method getLastStep
% Purpose: Determines the x coordinate of the last step based on a given x
%          coordinate. If currently on step, will take the previous step
%          climbed. Outputs 0 if no steps yet climbed.
% Parameters: 
%       obj -- the stair object whose to compare to the x coordinate.
%       x -- the x coordinate that will be compared to the stair step
%            locations.
% Returns: The x coordinate of the previously climbed step on the stair
%          object. If no steps have yet been climbed, returns 0.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function output = getLastStep(obj, x) 
            if (isOnStep(obj, x) == 1)
                output = (floor(x / obj.tread) - 1) * obj.tread;
            else
                output = floor(x / obj.tread) * obj.tread;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method isOnStairs
% Purpose: Determines whether the point input coordinate is located within
%          on the stairs or outside the stair line. The tolerance of this
%          function is defined with the 'tol' variable that has standard 
%          length units (see units in main.m)
% Parameters: 
%       obj -- the stair object which will be checked whether it contains
%              the input point.
%       x -- the x coordinate of the input point. 
%       y -- the y coordinate of the input point.
% Returns:
%       1 -- if the point is located on the stairs.
%       0 -- if the point is not located on the stairs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function output = isOnStairs(obj, x, y)
            tol = 0.0001;
            if (isOnStep(obj, x) == 1)
                high_end = (x / obj.tread) * obj.riser;
                low_end = high_end - obj.riser;
                hdif = y - high_end;
                ldif = low_end - y;
                if(hdif > tol || ldif > tol)
                    output = 0;
                else
                    output = 1;
                end
            else
                level = getLastStep(obj, x)*obj.riser;
                if(y ~= level) 
                    output = 1;
                else
                    output = 1;
                end
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method detectDomain
% Purpose: Provided a block number (see definitions in main.m), the upper
%          and lower x and y coordinates are provided in an array. In other
%          words, the smallest and largest x and y values are returned for
%          a given block. Since the zeroth block spans to negative
%          infinity its xmin output is meaningless.
% Parameters: 
%       obj -- the stair object whose block is investigated.
%       bnum -- the block number for the block whose coordinates will be
%               returned.
% Returns:
%       xmin - the smallest x coordinate that pertains to the input block.
%       xmax - the largest x coordinate that pertains to the input block.
%       ymin - the smallest y coordinate that pertains to the input block.
%       ymax - the largest y coordinate that pertains to the input block.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [xmin, xmax, ymin, ymax] = detectDomain(obj, bnum)
            largenum = 100000000000;
            xmin = bnum * obj.tread;
            xmax = xmin + obj.tread;
            ymin = bnum * obj.riser;
            ymax = ymin + obj.riser;
            if (bnum == 0)
                xmin = -largenum;
            end
        end
        
        function output = detectBlock(obj, x)
            bnum = ceil(x / obj.tread) - 1;
            if (bnum < 0) 
                bnum = 0;
            end
            output = bnum;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method isOnDomain
% Purpose: Provided a block number, a sub block number, as well as the
%          coordinates for a wheel (x center point, y center point and
%          radius), the method verifies if the wheel is located as the
%          specified stair block and sub block (see definitions in main.m).
% Parameters: 
%       obj -- the stair object in question.
%       x -- the x coordinate for the point to be verified.
%       y -- the y coordinate for the point to be verified.
%       bnum -- the block number for the block which will be checked.
%       sbnum -- the sub block number which will be checked.
%       R -- the radius of the wheel.
% Returns:
%       1 -- if the point is located on the sub block.
%       0 -- if the point is not located on the sub block.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Output = isOnDomain(obj, x, y, bnum, sbnum, R)
            [xmin, xmax, ymin, ymax] = obj.detectDomain(bnum);
            if (sbnum == 1) 
                if (((x > (xmax - R)) || (x < xmin)))
                    Output = 0;
                else
                    Output = 1;
                end
            elseif (sbnum == 2)
                if ((y > ymax) || (y < ymin + R))
                    Output = 0;
                else
                    Output = 1;
                end
            elseif (sbnum == 3)
                if ((abs(x - xmax) < 0.001) || (abs(y - ymax) < 0.001))
                    Output = 1;
                else 
                    Output = 0;
                end
            end
            if (x < xmax && bnum == 0 && sbnum == 1)
                Output = 1;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method draw_stairs
% Purpose: Generate a plot (y vs x) of the stairs. 
% Parameters: 
%       obj -- the stair object to be plotted.
%       length -- the maximum x coordinate to be drawn.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw_stairs(obj, length) %draws stairs from x = 0 to x = length
            x = linspace(0, length, 10000);
            y = (obj.riser ) * floor(x / obj.tread);
            plot(x, y);
        end
    end    
end