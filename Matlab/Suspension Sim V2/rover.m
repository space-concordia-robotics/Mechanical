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
        %The number of wheels used.
        %STRUCTURE: rnum
        rnum
        %Maximum length of the rover, assuming all configruations have to
        %comply with the same maximum length requirement.
        %STRUCTURE: Lmax
        lmax
        %The x coordinate of the center of mass.
        %STRUCTURE: xcm(configuration, iteration)
        xcm
        %The y coordinate of the center of mass 
        %STRUCTURE: ycm(configuration, iteration)
        ycm
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
        %STRUCTURE: warn(configuration)
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
        %The horizontal length of the trials. This is the final end point
        %of the trials in the x axis.
        %STRUCTURE: maxx
        maxx
        %The amount of iterations used per trial.
        %STRUTCTURE: res
        res
        %The maximum coefficient of friction required for a given trial
        %STRUCTURE: mew(configuration)
        mew
        %Array representing the mass matrix of the rover
        %STRUCTURE: m(amount of masses [usually wheels, bogie, chassis])
        m
        %The elements obtained from the force analysis
        %STRUCTURE: F(configuration, iteration, num of elements)
        F
        me
        mewlims
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
        Output = checkGeometry(obj, c);
        F = setupForceeqs(obj, c, i, mew, st, FN1, FN2, FN3, FRx1, FRx2, FRx3, FRy1, FRy2, FRy3, M1, M2, M3, FBx, FBy);
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method AssignGeometry
% Purpose: Defines the dimensions of the rover's geometry, including the
%          radius of each wheels.
% Parameters:
%       obj -- the rover object whose dimensions are modified.
%       dims -- an array containing all the dimensions for the rover.
%       Ra -- 1D array with three elements denoting the radius of each 
%             wheel starting the the front most and ending with the rear 
%             most.
%       c -- the configuration whose dimensions are defined.
% Returns: The object whose dimensions are defined.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = AssignGeometry(obj, dims, Ra, c)
                obj.dim(c, :) = dims;
                obj.R(c, :) = Ra;
        end
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
%       incDynamics -- A boolean that dictates whether force calculations
%                      are to be included or not.
% Returns: The rover object filling the numi sized dify array with
%          the difference between the ideal and actual y center of gravity
%          coordinates. The difyt value of the configuration is also
%          defined with the sum of all difyt values.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drive(obj, st, wnum, maxx, pitch, c, incDynamics)
           %Some initializations
           obj.difyt(c) = 0;
           obj.warn(c) = 0;
           obj.maxx = maxx;
           xi = st.tread;
           yi = obj.R(c,1);
           thi = pi/2;
           bnum = 0;
           sb = 2;
%            if(c ~= 1 && incDynamics == 1)
%                guess = obj.F(1, 1, :);
%            else
%                guess = zeros(1,4);
%                guess(1) = 0.5;
%                guess(2) = 100;
%                guess(3) = 100;
%                guess(4) = 100;
%            end
           %Determine important generic values
           totalblock = floor(maxx / st.tread);
           sbnum = 3*totalblock + (maxx - totalblock * st.tread) / st.tread;
           totalits = floor(sbnum * pitch);
           dx = (st.tread - obj.R(c, wnum)) / pitch;
           dy = (st.riser - obj.R(c, wnum)) / pitch;
           dth = pi/ (2 * pitch);
           obj.its = totalits;
           obj.mew(c, :, :) = [0, 10; 0, 0];
           for i = 1:totalits
               %Solve position and interpret results
               obj = DetectPos(obj, wnum, st, xi, yi, thi, c, i);
               if (obj.tch(c, i) == 0 || obj.val(c, i) == 0) 
                   obj.difyt(c) = [];
                   return;
               elseif (obj.warn(c) == 1)
                   disp("RISER WARNING");
                   obj.difyt(c) = [];
                   return;
               else
                   yt = st.slope * (obj.xcm(c, i) - obj.xcm(c, 1));
                   obj.dify(c, i) = ( yt - obj.ycm(i) )^2;
               end
               obj.difyt(c) = obj.difyt(c) + obj.dify(c, i);
               %Force analysis portion
               if(incDynamics == 1)
                   obj.F(c, i) = executeCalculations(obj, c, i);
                   %guess = obj.F(c, i, :);
                   if(obj.F(c, i) < obj.mew(c, 1, 2)) %MIN
                       obj.mew(c, 1, 1) = i;
                       obj.mew(c, 1, 2) = obj.F(c, i);
                   elseif(obj.F(c, i) > obj.mew(c, 2, 2)) %MAX
                       obj.mew(c, 2, 1) = i;
                       obj.mew(c, 2, 2) = obj.F(c, i);
                   end
               end
               %Determine next position of input wheel
               switch sb
                   case 1
                       xi = xi + dx;
                       yi = bnum * st.riser;
                       thi = 0;
                       if( xi - bnum * st.tread > st.tread - obj.R(wnum) )
                           xi = (bnum + 1)*st.tread;
                           yi = yi + obj.R(wnum);
                           thi = pi/2;
                           sb = 2;
                       end
                   case 2
                       yi = yi + dy;
                       xi = (bnum + 1) * st.tread;
                       thi = pi/2;
                       if(yi > (bnum + 1) *  st.riser)
                           xi = (bnum + 1) * st.tread;
                           yi = (bnum + 1) * st.riser;
                           sb = 3;
                       end
                   case 3
                       thi = thi - dth;
                       xi = (bnum + 1) * st.tread;
                       yi = (bnum + 1) * st.riser;
                       if (thi < 0)
                           bnum = bnum + 1;
                           sb = 1;
                           thi = 0;
                       end
               end
           end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Optimize
% Purpose: Based on input dimensional limits, a defined number of
%          configurations are created and a trial is run per configuration.
%          During each trial the vertical center of gravity difference is
%          calculated and the best and worse configurations are stored in
%          the min and max class properties, respectively. 
%          NOTE 2: A future force analysis module will be implemented to
%                  provide further optimization data in the future.
% Parameters:
%       obj -- the object to be optimized.
%       Wnum -- the wheel which will be used to loop through the stair as
%               the trial input.
%       st -- the stair used for the optimization.
%       Lrb -- an array of min and max possible values for dim(1).
%       Llb -- an array of min and max possible values for dim(2).
%       lrb -- an array of min and max possible values for dim(3).
%       llb -- an array of min and max possible values for dim(4).
%       db -- an array of min and max possible values for dim(5).
%       hb -- an array of min and max possible values for dim(6).
%       Rb -- An array for the wheel dimensions of the rover. The current
%             optimization strategy does not optimize wheel size.
%       in -- The number of iterations per dimension. In reality roughly
%             (in + 1)^(dimnum - 1) total iteraions will be run.
%       res -- the resolution of each trial. Each trial will consist of res
%              amount of snapshots on the stairs.
% Returns: The optimized object with its min and max properties filled.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Optimize(obj, Wnum, st, Rb, lims, in, res, incDynamics)
            %Currently, the function does not optimise for wheel size,
            %although it can easily be adjusted to do so, but the
            %assumption is that we are currently buying wheels and it is
            %outside of our control. Rb is thus just a 3 item array with
            %each wheel radius.
            obj.maxx = 1000;
            obj.res = res;
            it = (lims(:, 2) - lims(:, 1))/(in - 1);
            dimi = lims(:, 1);
            maxam = 0;
            minam = 999999999999999999;
            totalin = in^obj.dimnum + 1;
            for c = 1:totalin
                if (c ~= 1)
                    obj = obj.AssignGeometry(dimi, Rb, c);
                end
                %disp(c);
                allow = checkGeometry(obj, c);
                if(allow == 1)
                    obj = obj.Drive(st, Wnum, obj.maxx, res, c, incDynamics);
                    %Establish whether the trial is a max or min condition.
                    if( obj.difyt(c) > maxam)
                        obj.max = c;
                        maxam = obj.difyt(c);
                    elseif( obj.difyt(c) < minam)
                        obj.min = c;
                        minam = obj.difyt(c);
                    end
                    if( incDynamics == 1)
                        if( obj.mew(c, 2, 2) < obj.mewlims(2) )
                            obj.mewlims(1) = c;
                            obj.mewlims(2) = obj.mew(c, 2, 2);
                        end
                    end
                end
                %the block below takes care of incrementing the dimensions.
                for k = (obj.dimnum):-1:1
                    if( c ~= 1)
                        dimi(k) = dimi(k) + it(k);
                            if( dimi(k) > lims(k, 2) )
                                dimi(k) = lims(k, 1);
                            else
                                break;
                            end
                    end
                end
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Animate
% Purpose: Create an animation of the rover composed of flicking through
%          the plots of the location of the rover of each iteration.
% Parameters: 
%       obj -- The rover object that will be animated.
%       c -- The rover configuration to animate.
%       st -- the stair object which will be shown in the animation. 
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Animate(obj, c, st)
            for i = 1:obj.its
               hold on;
               st.draw_stairs(1.25 * obj.x(c, obj.its, 1) );
               obj.draw(c, i);
               hold off;
               pause(0.25);
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method appAnimate
% Purpose: Create an animation of the rover composed of flicking through
%          the plots of the location of the rover of each iteration. The
%          animation is displayed on a given app GUI.
% Parameters: 
%       obj -- The rover object that will be animated.
%       app - the app window where the animation will be displayed.
%       st -- the stair object which will be shown in the animation. 
%       c -- The rover configuration to animate.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function appAnimate(obj, app, st, c)
            hold(app.UIAxes, 'on');
            for i = 1:obj.its
               cla(app.UIAxes);
               st.app_draw(app, 1.25 * obj.x(c, obj.its, 1) );
               obj.appdraw(app, c, i);
               pause(obj.res/360);
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
            xmin(1) = obj.xcm(obj.min, 1);
            xmin(2) = obj.xcm(obj.min, 1) + maxx;
            ymin(1) = obj.ycm(obj.min, 1);
            ymin(2) = (xmin(2) - xmin(1))*st.slope + ymin(1);
            xmax(1) = obj.xcm(obj.max, 1);
            xmax(2) = obj.xcm(obj.max, 1) + maxx;
            ymax(1) = obj.ycm(obj.max, 1);
            ymax(2) = (xmax(2) - xmax(1))*st.slope + ymax(1);
            plot(xmin, ymin,'DisplayName','Min ideal line');
            plot(obj.xcm(obj.min,:), obj.ycm(obj.min,:),'DisplayName', 'Min actual line');
            plot(xmax, ymax,'DisplayName','Max ideal line');
            plot(obj.xcm(obj.max,:), obj.ycm(obj.max,:),'DisplayName', 'Max actual line');
            hold off;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method plotCG
% Purpose: Generates a plot of the CG in an app UI.
% Parameters: 
%       obj -- The rover object whose CG will be plotted.
%       app -- The app where the plot will be displayed.
%       st -- The stair used in the optimization.
%       isIdeal -- will either plot the ideal CG path or the one based on
%                  the trial depending on whether this variable is true or
%                  false, respectively.
%       c -- The configuration whose CG will be plotted.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotCG(obj, app, st, isIdeal, c)
            if(isIdeal == 1)
                xp(1) = obj.xcm(c, 1);
                xp(2) = obj.xcm(c, 1) + obj.maxx;
                yp(1) = obj.ycm(c, 1);
                yp(2) = obj.ycm(c, 1) + (xp(2) - xp(1))*st.slope;
                plot(app.UIAxes, xp, yp);
            elseif(isIdeal == 0)
                plot(app.UIAxes, obj.xcm(c,:), obj.ycm(c,:));
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function x = executeCalculations(obj, c, i)
            %options = optimoptions('fsolve','MaxIterations',5000000);
            %options.MaxFunctionEvaluations = 5000000;
            %Func = @(input) setupForceeqs(obj, c, i, st, input);
            %z = fsolve(Func, guess, options);
            a(1) = obj.dim(c, 4) * cos( obj.alpha(c, i) ) + obj.dim(c, 5) * sin( obj.alpha(c, i) );
            a(2) = obj.dim(c, 3) * cos( obj.alpha(c, i) ) - obj.dim(c, 5) * sin( obj.alpha(c, i) );
            a(3) = obj.dim(c, 5) * cos( obj.alpha(c, i) ) - obj.dim(c, 4) * sin( obj.alpha(c, i) );
            a(4) = obj.dim(c, 5) * cos( obj.alpha(c, i) ) + obj.dim(c, 3) * sin( obj.alpha(c, i) );
            b(1) = (obj.dim(c, 1) + obj.dim(c, 2)) * cos(obj.beta(c, i)) - obj.dim(c, 5) * sin(obj.beta(c, i));
            b(2) = obj.dim(c, 5) * cos(obj.beta(c, i)) + (obj.dim(c, 1) + obj.dim(c, 2)) * sin(obj.beta(c, i));
            b(3) = obj.dim(c, 1) * cos(obj.beta(c, i)) - obj.dim(c, 6) * sin(obj.beta(c, i));
            lh(1) = a(2) * sin( obj.th(c, i, 2) ) - a(4) * cos( obj.th(c, i, 2) ) - obj.R(c, 2);
            lh(2) = a(2) * cos( obj.th(c, i, 2) ) + a(4) * sin( obj.th(c, i, 2) );
            rh(1) = a(1) * sin( obj.th(c, i, 1) ) + a(3) * cos( obj.th(c, i, 1) ) + obj.R(c, 1);
            rh(2) = a(1) * cos( obj.th(c, i, 1) ) - a(3) * sin( obj.th(c, i, 1) );
            k(1) = sin( obj.th(c, i, 3) - obj.th(c, i, 1) ) + (b(1) / b(3)) * sin( obj.th(c, i, 1)) * cos( obj.th(c, i, 3) ) - (b(2) / b(3)) * cos( obj.th(c, i, 1) ) * cos( obj.th(c, i, 3) ) - obj.R(c, 3) * cos( obj.th(c, i, 1) );
            k(2) = (b(1) / b(3)) * cos( obj.th(c, i, 1) + obj.th(c, i, 3) ) + (b(2) / b(3))* sin( obj.th(c, i, 1) + obj.th(c, i, 3) ) + obj.R(c, 3) * sin( obj.th(c, i, 1) );
            k(3) = sin( obj.th(c, i, 3) - obj.th(c, i, 1) ) - (b(1) / b(3)) * cos( obj.th(c, i, 1) ) * sin( obj.th(c, i, 3) ) - (b(2) / b(3)) * sin( obj.th(c, i, 1) ) * sin( obj.th(c, i, 3) );
            j(1) = sin( obj.th(c, i, 2) - obj.th(c, i, 3) ) + (obj.R(c, 3) / b(3)) * cos( obj.th(c, i, 2) ) - (b(1) / b(3)) * sin( obj.th(c, i, 2) ) * cos( obj.th(c, i, 3) ) + (b(2) / b(3)) * cos( obj.th(c, i, 2) ) * cos( obj.th(c, i, 3) );
            j(2) = - (b(1) / b(3)) * cos( obj.th(c, i, 2) + obj.th(c, i, 3) ) - (b(2) / b(3)) * sin( obj.th(c, i, 2) - obj.th(c, i, 3) ) - (obj.R(c, 3) / b(3) ) * sin( obj.th(c, i, 2) );
            j(3) = sin( obj.th(c, i, 2) - obj.th(c, i, 3) ) + (b(1) / b(3)) * cos( obj.th(c, i, 2) ) * sin( obj.th(c, i, 3) ) + (b(2) / b(3)) * sin( obj.th(c, i, 2) ) * sin( obj.th(c, i, 3) );
%             k(1) = -sin( obj.th(c, i, 3) - obj.th(c, i, 1) ) * cos( obj.th(c, i, 3) );
%             k(2) = sin( obj.th(c, i, 3) ) * sin( obj.th(c, i, 3) - obj.th(c, i, 1)) - (b(1) / b(3)) * sin( obj.th(c, i, 1) ) * cos ( obj.th(c, i, 3) ) - (b(2) / b(3)) * cos( obj.th(c, i, 1) ) * cos( obj.th(c, i, 3) );
%             k(3) = obj.R(c, 3) / b(3) - (b(1) / b(3)) * cos( obj.th(c, i, 1) + obj.th(c, i, 3) )+ (b(2) / b(3)) * sin( obj.th(c, i, 1) + obj.th(c, i, 3) ) - sin ( obj.th(c, i, 3) - obj.th(c, i, 1) ) * cos( obj.th(c, i, 3) );
%             k(4) = sin( obj.th(c, i, 3) ) * sin( obj.th(c, i, 3) - obj.th(c, i, 1)) + (b(1) / b(3)) * cos( obj.th(c, i, 1)) * sin( obj.th(c, i, 3) ) - (b(2) / b(3)) * sin( obj.th(c, i, 1) ) * sin( obj.th(c, i, 3) );
%             j(1) = cos( obj.th(c, i, 3) ) * (cos( obj.th(c, i, 2) + obj.th(c, i, 3) ) + (b(2) / b(3)) * cos( obj.th(c, i, 2) ) * cos( obj.th(c, i, 3) ) + (b(1) / b(3)) * sin( obj.th(c, i, 2) ) * sin( obj.th(c, i, 3) ) - obj.R(c, 3) * cos( obj.th(c, i, 2) ) / b(3) );
%             j(2) = obj.R(c, 3) * sin( obj.th(c, i, 2) + obj.th(c, i, 3) ) / b(3) - sin( obj.th(c, i, 3) ) * cos( obj.th(c, i, 2) + obj.th(c, i, 3) ) - (b(2) / b(3)) * cos( obj.th(c, i, 3) ) * (cos( obj.th(c, i, 2) ) * sin( obj.th(c, i, 3) ) + sin( obj.th(c, i, 3) - obj.th(c, i, 2) )) + (b(1) / b(3)) * (cos( obj.th(c, i, 2) + obj.th(c, i, 3) ) * cos( obj.th(c, i, 3) ) - sin( obj.th(c, i, 2) ) * sin( obj.th(c, i, 3) )^2 );
%             j(3) = sin( obj.th(c, i, 3) ) * ((b(2) / b(3)) * (sin( obj.th(c, i, 3) - obj.th(c, i, 2) ) - sin( obj.th(c, i, 2) ) * cos( obj.th(c, i, 3) )) - (b(1) / b(3)) * (cos( obj.th(c, i, 3) + obj.th(c, i, 2) ) + cos( obj.th(c, i, 2) ) * cos( obj.th(c, i, 3) ) ) - obj.R(c, 3) * sin( obj.th(c, i, 2) ) / b(3) + cos( obj.th(c, i, 2) + obj.th(c, i, 3) ) );
%             j(4) = sin( obj.th(c, i, 3) )^2 * ( (b(1) / b(3)) * cos( obj.th(c, i, 2) ) + (b(2) / b(3)) * sin( obj.th(c, i, 2) ) - cos( obj.th(c, i, 2) ) + sin( obj.th(c, i, 2) ) );
            xai(1) = lh(1) * k(1) - rh(1) * j(1);
            xai(2) = lh(2) * k(1) + lh(1) * k(2) - rh(2) * j(1) - rh(1) * j(2);
            xai(3) = lh(2) * k(2) + lh(1) * k(3) - rh(2) * j(2) - rh(1) * j(3);
            xai(4) = lh(2) * k(3) - rh(2) * j(3);
            obj.me(c, i, :) = roots(xai);
            x = obj.me(c, i, 2);
            for d = 1:3
                if(isreal( obj.me(c, i, d) ) && obj.me(c, i, d) > 0 && obj.me(c, i, d) < 1.0 )
                    x = obj.me(c, i, d);
                    break;
                elseif(isreal( obj.me(c, i, d) ) && ( obj.me(c, i, d) < 0.001 && obj.me(c, i, d) > -0.001) )
                    x = 0.0;
                end
            end
            %x = [z, yi.'];
        end
    end
end