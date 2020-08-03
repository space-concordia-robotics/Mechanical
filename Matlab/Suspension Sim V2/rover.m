%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rover class
% Author: Maxim Kaller
% Purpose: An abstact class to include higher level driving methods.
%          Subclasses of specific rover geometries are developped to deal
%          with specific kinematic and dynamic methods. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef (Abstract) rover
    properties
        %This array stored dimensional values specific to each rover 
        %subclass. It is up to the subclasses to mange this property.
        %STRUCTURE: dim(configuration)
        dim 
        %The sizse of the dim class property, assuming all configurations
        %have the same number of dimensions.
        %STRUCTURE: dimnum
        dimnum 
        %An array of wheel radii. 
        %STRUCTURE: R(configuration, wheel number)
        R 
        %Maximum length of the rover, assuming all configruations have to
        %comply with the same maximum length requirement.
        %STRUCTURE: Lmax
        lmax
        %The x coordinate of the center of mass.
        %STRUCTURE: xcm(configuration, iteration)
        xcm
        %The x coordinate of the center of mass of the starting point of
        %the iteration. NOTE: This property will be removed from subsequent
        %releases of the script. 
        %STRUCTURE: xcmo(configuration)
        xcmo
        %The y coordinate of the center of mass 
        %STRUCTURE: ycm(configuration, iteration)
        ycm 
        %The y coordinate of the center of mass of the starting point of
        %the iteration. NOTE: This property will be removed from subsequent
        %releases of the script. 
        %STRUCTURE: ycmo(configuration)
        ycmo
        %An array of the x coordinates of each wheel location.
        %STRUCTURE: x(configuration, iteration, wheel number)
        x 
        %An array of the y coordinates of each wheel location.
        %STRUCTURE: y(configuration, iteration, wheel number)
        y 
        %An array of the contact angle of each wheel.
        %STRUCTURE: th(configuration, iteration, wheel number)
        th 
        %Total check: A variable that checks that all rover contact points 
        %are on the road.
        %STRUCTURE: tch(configuration, iteration)
        tch
        %A variable that confirms whether a valid solution has been
        %calculated for the rover's positioning.
        %STRUCTURE: val(configuration, iteration)
        val
        %Difference between expected and actual y coordinate of the rover's
        %center of gravity.
        %STRCUTURE: dify(configuration, iteration)
        dify 
        %Sum of all dify values for a given instance (see definition of 
        %instance in main.m).
        %STRUCTURE: difyt(configuration)
        difyt
        %This property is true when the configuration is shown to have 
        %undesired effects during a trial. A solution with a warning 
        %differs from an invalid solution in that an invalid solution 
        %cannot exist in the real world whereas a solution with a warning 
        %can exist, but is simply not desired.
        %STRUCTURE: warn(configuration, iteration)
        warn
        %The configuration where difyt is the higest.
        %STRUCTURE: max
        max
        %The configuration where difyt is the smallest.
        %STRUCTURE: min
        min
        %The number of instances solved per configuration.
        %STRUCTURE: its
        its
    end
    
    methods (Abstract)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method DetectPos
% Purpose: For driving on a stair, calculate the overall configuration of a
%          rover for a particular instance. This is caulcated based of 
%          location coordinate inputs of a given wheel center as well as 
%          the wheel's contact angle. This function is particular to each 
%          given subclass. Look at the particular subclass function for 
%          more info.
% Parameters: 
%       obj -- The rover object whose kinematic configuration will be
%              calculated.
%       Wheelnum -- The wheel whose coordinates are provided. 1 being the
%                   frontmost wheel and each following wheel incrementing 
%                   by 1.
%       stair -- The stair object that acts as the road.
%       x -- The x coordinate for the input wheel's center.
%       y -- The y coordinate for the input wheel's center.
%       th -- The contact angle for the input wheel.
%       c -- The rover configuration whose position is determined.
%       i -- the particular instance in question.
% Returns: The rover object with modified x, y and th values as well as any
% subclass specific values.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        obj = DetectPos(obj, Wheelnum, stair, x, y, th, c, i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Optimize
% Purpose: For driving on a stair, loop through all possible configurations
%          to determine the optimal set of rover dimensions for the task.
%          Based on the iteration number input, as well as the dimension
%          limits, (in)^(dnum) configurations are established and their
%          properties are calculated. The configurations with the best and
%          worst properties are saved.
% Parameters: 
%       obj -- The rover object to be optimized.
%       Wheelnum -- The wheel which will be used as input for the trials of
%                   each configuration, 1 being the frontmost wheel and 
%                   each following wheel incrementing by 1.
%       st -- The stair object that acts as the road during all
%             optimization trials.
% Returns: The rover object with max and min parameters indicating the
%          configuration number with the worst and best configurations,
%          respectively.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        obj = Optimize(obj, Wnum, st, Lrb, Llb, lrb, llb, db, hb, Rb, in, res);
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Drive
% Purpose: Create numi number of iterations spread across the st stair
%          object. For each iteration, the location of the rover is
%          determined by a location of wheel number wnum. The wheel begins
%          at the start of the stairs and advances forward after each
%          iteration until x = maxx is reached.
% Parameters: 
%       obj -- The rover object driven through the stairs.
%       st -- The stair object that the rover will drive through.
%       wnum -- The wheel number which will move along the stairs and act
%               as the input for the rover's position.
%       maxx -- The maximum x coordinate that the wnum wheel will travel
%               to. NOTE: x can only be on the stair's tread.
%       numi -- The amount of iterations used during driving. The number of
%               iterations determine the distance between each iteration.
%       c -- The configuration used for the rover object.
% Returns: The rover object filling the numi sized dify array with
%          the difference between the ideal and actual y center of gravity
%          coordinates. The difyt value of the configuration is also
%          defined with the sum of all difyt values.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drive(obj, st, wnum, maxx, numi, c)
           obj.difyt(c) = 0;
           obj.warn(c) = 0;
           xi = st.tread - obj.R(c, 1);
           yi = 0;
           thi = 0;
           bnum = floor(maxx / st.tread);
           sbnum = 3*bnum + (maxx - bnum * st.tread) / st.tread;
           ith = floor( numi/ sbnum );
           dx = (st.tread - obj.R(c, wnum)) / ith;
           dy = (st.riser - obj.R(c, wnum)) / ith;
           dth = pi/ (2 * ith);
           obj.its = numi;
           cas = 0;
           for i = 1:numi
               itnum = floor(i / ith);
               bnum = floor(itnum / 3);
               sbi = mod(itnum, 3) + 1;
               switch sbi
                   case 1
                       cas = 1;
                       xi = xi + dx;
                       yi = bnum * st.riser;
                       thi = 0;
                   case 2
                       if ( cas ~= 2)
                           yi = yi + obj.R(c, wnum);
                       end
                       cas = 2;
                       yi = yi + dy;
                       xi = (bnum + 1) * st.tread;
                       thi = pi/2;
                   case 3
                       if (cas ~= 3)
                           thi = pi /2;
                       end
                       cas = 3;
                       thi = thi - dth;
                       xi = (bnum + 1) * st.tread;
                       yi = (bnum + 1) * st.riser;
               end
               obj = DetectPos(obj, wnum, st, xi, yi, thi, c, i);
               if (i == 1)
                   obj.xcmo(c) = obj.xcm(c, 1);
                   obj.ycmo(c) = obj.ycm(c, 1);
               end
               if (obj.tch(c, i) == 0 || obj.val(c, i) == 0) 
                   obj.difyt(c) = [];
                   return;
               elseif (obj.warn(c) == 1)
                   disp("RISER WARNING");
                   obj.difyt(c) = [];
                   return;
               else
                   yt = st.slope * (obj.xcm(c, i) - obj.xcmo(c));
                   obj.dify(c, i) = ( yt - obj.ycm(i) )^2;
               end
               obj.difyt(c) = obj.difyt(c) + obj.dify(c, i);
           end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Animate
% Purpose: Create an animation of the rover composed of flicking through
%          the plots of the location of the rover of each iteration.
% Parameters: 
%       obj -- The rover object that will be animated.
%       c -- The rover configuration to animate.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Animate(obj, c)
            for i = 1:obj.its
               hold on;
               st.draw_stairs(1.25 * obj.x(c, obj.its, 1) );
               obj.draw(c, i);
               hold off;
               pause(0.04);
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method CompareCG
% Purpose: Generates a plot comparing the ideal and actual cg locations for
%          the best and worst trials. A total of four curves are plotted.
% Parameters: 
%       obj -- The rover object that will be animated.
%       c -- The rover configuration to animate.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function CompareCG(obj, st, maxx)
            hold on;
            xmin(1) = obj.xcmo(obj.min);
            xmin(2) = obj.xcmo(obj.min) + maxx;
            ymin(1) = obj.ycmo(obj.min);
            ymin(2) = (xmin(2) - xmin(1))*st.slope + ymin(1);
            xmax(1) = obj.xcmo(obj.max);
            xmax(2) = obj.xcmo(obj.max) + maxx;
            ymax(1) = obj.ycmo(obj.max);
            ymax(2) = (xmax(2) - xmax(1))*st.slope + ymax(1);
            plot(xmin, ymin,'DisplayName','Min ideal line');
            plot(obj.xcm(obj.min,:), obj.ycm(obj.min,:),'DisplayName', 'Min actual line');
            plot(xmax, ymax,'DisplayName','Max ideal line');
            plot(obj.xcm(obj.max,:), obj.ycm(obj.max,:),'DisplayName', 'Max actual line');
            hold off;
        end
    end
end