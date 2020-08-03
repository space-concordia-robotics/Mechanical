%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rockerbogie class
% Author: Maxim Kaller
% Purpose: A rover subclass to implement methods specific to the rocker
%          bogie kinematic configuration of a Mars rover. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef rockerbogie < rover
    %Refer to RockerBogie.PNG for a description of the dimension numbering
    properties
        %The angle the front bogie makes with the x axis.
        alpha
        %The angle the rocker makes with the x axis.
        beta
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method rockerbogie (constructor)
% Purpose: Defines the class properties and dimensions based on the inputs. 
% Parameters: See the AssignGeometry method - The properties are the same.
% Returns: The object which is being initialized.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = rockerbogie(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Ra, lmax)
            AssignGeometry(obj, rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Ra, 1);
            obj.difyt = 0;
            obj.lmax = lmax;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method AssignGeometry
% Purpose: Defines the dimensions of the rockerbogie's geometry.
% Parameters:
%       obj -- the rockerbogie object to modify.
%       rocker_left -- the length of the rocker to the left of the centre
%                      of gravity. The first item in the dim property 
%                      (rover class property).
%       rocker_right -- the length of the rocker to the right of the centre
%                       of gravity. The second item in the dim property
%                       (rover class property).
%       bogie_left -- the length of the bogie to the left of the bogie's
%                     pivot point. The third item in the dim property
%                     (rover class property).
%       bogie_right -- the length of the bogie to the right of the bogie's
%                      pivot point. The fourth item in the dim property
%                      (rover class property).
%       bogie_height -- when the rover sits on flat ground, the vertical
%                       distance between the center of the wheels and the
%                       bogie's pivot point. The fifth item in the dim
%                       property (rover class property).
%       cg_height -- when the rover sits on flat ground, the vertical
%                    distance between the center of the wheels and the
%                    rover's centre of gravity. The sixth item in the dim
%                    property (rover class property).
%       Ra -- 1D array with three elements denoting the radius of each 
%             wheel starting the the front most and ending with the rear 
%             most.
%       c -- the configuration whose dimensions are defined.
% Returns: The object whose dimensions are defined.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = AssignGeometry(obj, rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Ra, c)
            %Defines geometric properties only
            obj.dim(c, 1) = rocker_left;
            obj.dim(c, 2) = rocker_right;
            obj.dim(c, 3) = bogie_left;
            obj.dim(c, 4) = bogie_right;
            obj.dim(c, 5) = bogie_height;
            obj.dim(c, 6) = cg_height;
            obj.R(c,1) = Ra(1);
            obj.R(c,2) = Ra(2);
            obj.R(c,3) = Ra(3);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method DetectPos
% Purpose: Uses a variety of methods to detect the full position and the
%          orientation of the rover. Loops through block and sub block
%          numbers for all non input wheels to determine the right
%          combination.
% Parameters:
%       obj -- the rockerbogie object whose position will be determined.
%       Wheelnum -- the number corresponding to the input wheel.
%       stair -- the stair object on which the rover is driving.
%       x -- x coordinate input for the wheel contact point on the stair.
%       y -- y coordinate input for the wheel contact point on the stair.
%       th -- wheel contact angle input (see definitions in main.m for
%             information on contact angles).
%       c -- the configuration whose position is being calculated.
%       i -- the iteration whose position is being calculated.
% Returns: The rockerbogie object with defined coordinate and orientation
%          properties.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = DetectPos(obj, Wheelnum, stair, x, y, th, c,  i)
            if( stair.isOnStairs(x, y) == 0)
                disp('ERROR: configurePos input not on stair');
                return;
            end
            largenum = 10000;
            bnum = stair.detectBlock(x);
            iblk = 4;
            solfound = 0;
            switch Wheelnum
                %This assigns basic loop information based on the number of
                %the input wheel.
                case 1
                    e1 =  0;
                    e2 =  0;
                    i1 = -1;
                    i2 = -1;
                    w1 =  2;
                    w2 =  3;
                case 2
                    e1 =  largenum;
                    e2 =  0;
                    i1 =  1;
                    i2 = -1;
                    w1 =  1;
                    w2 =  3;
                case 3
                    e1 = largenum;
                    e2 = largenum;
                    i1 = 1;
                    i2 = 1;
                    w1 = 1;
                    w2 = 2;
                otherwise
                    disp('Error in Distloop. Incorrect Wheelnum');
            end
            for l1 = bnum:i1:e1
                if (solfound == 1)
                    break;
                end
                for s1 = 3:-1:1
                    if (solfound == 1)
                        break;
                    end
                    for l2 = l1:i2:e2
                        if (solfound == 1)
                            break;
                        end        
                        for s2 = 3:-1:1
                            blk(w1) = l1;
                            blk(w2) = l2;
                            blk(Wheelnum) = bnum;
                            sb(w1) = s1;
                            sb(w2) = s2;
                            sb(Wheelnum) = iblk;
                            if (Wheelnum ~= 3)
                                obj = obj.cForm( stair, Wheelnum, x, y, th, blk, sb, c, i);
                            else
                                obj = genericSolver(obj, stair, blk, sb, x, y, th, c, i);
                            end
                            if(obj.tch(c, i) == 1 && obj.val(c, i) == 1)
                                solfound = 1;
                                if(sb(1) == 2 && sb(2) == 2)
                                    obj.warn(c) = 1;
                                end
                                break;
                            end
                        end
                    end
                end
            end
            if(solfound == 0)
                disp('ERROR: Solution not found!');
                return;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method genericSolver
% Purpose: A general method to solve for the position and orientation of
%          the rover with the block and sub block numbers of all wheels as
%          well as the coordinates of the contact point of any wheel.
%          NOTE: The function is currently unused and will only be
%                reintegrated once a rear wheel mode is reintroduced to
%                define the rover's position with rear wheel coordinate
%                inputs. The method used right now that accepts front and 
%                mid wheel inputs is closed form in nature and functions at
%                a much faster rate than solving this set of equations.
% Parameters:
%       obj -- the rockerbogie object whose position and orientation will
%              be solved.
%       stair -- the stair used to solve for the position and orientation
%                of the rover.
%       blck -- a 1D array of three elements denoting the block number for
%            each wheel (see the definitions in main.m for more information
%            on block numbers).
%       sub -- a 1D array of three elements denoting the sub block number 
%              for each wheel (see the definitions in main.m for more 
%              information on sub block numbers).
%       x -- x coordinate input for the wheel contact point on the stair.
%       y -- y coordinate input for the wheel contact point on the stair.
%       th -- wheel contact angle input (see definitions in main.m for
%             information on contact angles).
%       c -- the configuration for whom the equations are being generated.
%       i -- the iteration where the results will be saved.
% Returns: The objects with defined coordinate and orientation proerties.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = genericSolver(obj, stair, blck, sub, x, y, th, c, i)
            obj.tch = 0;
            sol = obj.genStairEq(stair, blck, sub(1), sub(2), sub(3), x, y, th, c);
            if(isempty(sol.prx) || isempty(sol.pmx) || isempty(sol.pfx))
                disp('ERROR: vpasolve error. Empty values. Following inputs:');
                disp(blck);
                disp(sub);
                obj.x = [];
            else
                %prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
                obj.alpha(c,i) = sol.alp;
                obj.beta(c,i) = sol.bet;
                obj.x(c,i, 3) = sol.prx - obj.R(3) * sin(sol.thr);
                obj.y(c,i, 3) = sol.pry + obj.R(3) * cos(sol.thr);
                obj.x(c,i, 2) = sol.pmx - obj.R(2) * sin(sol.thm);
                obj.y(c,i, 2) = sol.pmy + obj.R(2) * cos(sol.thm);
                obj.x(c,i, 1) = sol.pfx - obj.R(1) * sin(sol.thf);
                obj.y(c,i, 1) = sol.pfy + obj.R(1) * cos(sol.thf);
                obj.tch(c, i) = tcheck(obj, stair, blck, sub, obj.x, obj.y);
                obj.xcm(c, i) = sol.prx + obj.Ll*cos(sol.bet) - (obj.delta + obj.hc)*sin(sol.bet);
                obj.ycm(c, i) = sol.pry + obj.Ll*sin(sol.bet) + (obj.delta + obj.hc)*cos(sol.bet);
                obj.th(c,i,1) = sol.thf;
                obj.th(c,i,2) = sol.thm;
                obj.th(c,i,3) = sol.thr;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method genStairEq
% Purpose: Defines a set of equations to describe the rover's given
%          kinematic state.
%          NOTE: The function is currently unused and will only be
%                reintegrated once a rear wheel mode is reintroduced to
%                define the rover's position with rear wheel coordinate
%                inputs. The method used right now that accepts front and 
%                mid wheel inputs is closed form in nature and functions at
%                a much faster rate than solving this set of equations.
% Parameters:
%       obj -- the rockerbogie object whose properties will be used to
%              generate the set of equations.
%       stair -- the stair whose properties will be used to generate the
%                set of equations.
%       b -- a 1D array of three elements denoting the block number for
%            each wheel (see the definitions in main.m for more information
%            on block numbers).
%       bbi -- the sub block number of wheel i (see the definitions in
%              main.m for more information on sub blocks).
%       xi -- x coordinate input for the wheel contact point on the stair.
%       yi -- y coordinate input for the wheel contact point on the stair.
%       thi -- wheel contact angle input (see definitions in main.m for
%              information on contact angles).
%       c -- the configuration for whom the equations are being generated.
% Returns: A set of 13 equations that fully describe the position and
%          orientation of the rover based on geometric properties and wheel
%          coordinate inputs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function F = genStairEq(obj, stair, b, bb1, bb2, bb3, xi, yi, thi, c)
            %bbi = 1 -> flat ground 
            %bbi = 2 -> step
            %bbi = 3 -> corner
            %bbi = 4 -> input
            lc = obj.dim(c, 1) + obj.dim(c, 2);
            inpt = 0;
            syms prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
            eq1 = prx - obj.R(c,3) * sin(thr) - obj.dim(c, 5) * sin(bet) + lc * cos(bet) == ppx;
            eq2 = pry + obj.R(c,3) * cos(thr) + obj.dim(c, 5) * cos(bet) + lc * sin(bet) == ppy;
            eq3 = pmx - obj.R(c,2) * sin(thm) - obj.dim(c, 5) * sin(alp) + obj.dim(c, 3) * cos(alp) == ppx;
            eq4 = pmy + obj.R(c,2) * cos(thm) + obj.dim(c, 5) * cos(alp) + obj.dim(c, 3) * sin(alp) == ppy;
            eq5 = pfx - obj.R(c,1) * sin(thf) - obj.dim(c, 5) * sin(alp) - obj.dim(c, 4) * cos(alp) == ppx;
            eq6 = pfy + obj.R(c,1) * cos(thf) + obj.dim(c, 5) * cos(alp) - obj.dim(c, 4) * sin(alp) == ppy;
            switch bb1
                case 1
                    eq7  = b(1) * stair.riser       == pfy;
                    eq8  = thf == 0;
                case 2
                    eq7  = (b(1) + 1) * stair.tread == pfx;
                    eq8  = thf == pi/2;
                case 3
                    eq7  = (b(1) + 1) * stair.tread == pfx;
                    eq8  = (b(1) + 1) * stair.riser == pfy;
                case 4
                    eq11 = xi  == pfx;
                    eq12 = yi  == pfy;
                    eq13 = thf == thi;
                    inpt = 1;
                otherwise
                    disp('ERROR: Wrong bb1');
            end
            switch bb2
                case 1
                    if (inpt == 0)
                        eq9  = b(2) * stair.riser       == pmy;
                        eq10 = thm == 0;
                    else
                        eq7  = b(2) * stair.riser       == pmy;
                        eq8  = thm == 0;   
                    end
                case 2
                    if (inpt == 0)   
                        eq9  = (b(2) + 1) * stair.tread == pmx;
                        eq10 = thm == pi/2;
                    else
                        eq7  = (b(2) + 1) * stair.tread == pmx;
                        eq8  = thm == pi/2;                        
                    end
                case 3
                    if (inpt == 0)
                        eq9  = (b(2) + 1)*stair.tread   == pmx;
                        eq10 = (b(2) + 1)*stair.riser   == pmy;
                    else
                        eq7  = (b(2) + 1)*stair.tread   == pmx;
                        eq8  = (b(2) + 1)*stair.riser   == pmy;
                    end
                case 4
                    eq11  = xi  == pmx;
                    eq12  = yi  == pmy;
                    eq13  = thm == thi;
                    inpt  = 1;
                otherwise
                    disp('ERROR: Wrong bb2');
            end
            switch bb3
                case 1
                    eq9   = b(3) * stair.riser       == pry;
                    eq10  = thr == 0;
                case 2
                    eq9   = (b(3) + 1) * stair.tread == prx;
                    eq10  = thr == pi/2;
                case 3
                    eq9   = (b(1) + 1)*stair.tread   == prx;
                    eq10  = (b(1) + 1)*stair.riser   == pry;
                case 4
                    eq11  = xi  == prx;
                    eq12  = yi  == pry;
                    eq13  = thr == thi;
                    inpt  = 1;
            end
            if (inpt == 0)
                disp('ERROR: Double check that wheel input has been properly given');
            end
            F = vpasolve([eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12, eq13], [prx, pry, pmx, pmy, pfx, pfy, thr, thm, thf, alp, bet, ppx, ppy]);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method cForm
% Purpose: A more specific, closed form method to solve for the position 
%          and orientation of the rover with the block and sub block 
%          numbers of all wheels as well as the coordinates of the contact 
%          points of EITHER the front wheel or mid wheel.
% Parameters:
%       obj -- the rockerbogie object whose position and orientation will
%              be solved.
%       st -- the stair used to solve for the position and orientation of 
%             the rover.
%       Wg -- The input wheel number.
%       xs -- x coordinate input for the wheel contact point on the stair.
%       ys -- y coordinate input for the wheel contact point on the stair.
%       ts -- wheel contact angle input (see definitions in main.m for
%             information on contact angles).
%       blk -- a 1D array of three elements denoting the block number for
%            each wheel (see the definitions in main.m for more information
%            on block numbers).
%       sblk -- a 1D array of three elements denoting the sub block number 
%              for each wheel (see the definitions in main.m for more 
%              information on sub block numbers).
%       c -- the configuration for whom the equations are being generated.
%       i -- the iteration where the results will be saved.
% Returns: The objects with defined coordinate and orientation proerties.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = cForm(obj, st, Wg, xs, ys, ts, blk, sblk, c, i)
           xg = xs - obj.R(c, Wg)*sin(ts);
           yg = ys + obj.R(c, Wg)*cos(ts);
           lb = obj.dim(c, 3) + obj.dim(c, 4);
           lo = obj.dim(c, 1) + obj.dim(c, 2);
           rr = sqrt ( (obj.dim(c, 5))^2 + lo^2);
           if (Wg ~= 3)
               sign = (-1)^Wg;
               Wo = 3 - Wg;
               switch sblk(Wo)
                   case 1
                       yov = blk(Wo) * st.riser;
                       yo  = yov + obj.R(c, Wo);
                       alp = asin( (yg - yo)/ lb );
                       xo  = xg + sign * lb * cos(alp);
                       xov = xo;
                       tho = 0;
                       if( ~isreal(alp) || isempty(alp) || isnan(alp))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
                   case 2
                       xov = (blk(Wo) + 1) * st.tread;
                       xo  = xov - obj.R(c, Wo);
                       alp = acos( (xg - xo)/ lb );
                       yo  = yg + sign * lb * sin(alp);
                       yov = yo;
                       tho = pi/2;
                       if( ~isreal(alp) || isempty(alp) || isnan(alp))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
                   case 3 
                       yov = (blk(Wo) + 1) * st.riser;
                       xov = (blk(Wo) + 1) * st.tread; 
                       p = sqrt( (xg - xov)^2 + (yg - yov)^2 );
                       line = (obj.R(c, Wo)^2 + p^2 - lb^2) / (2*obj.R(c, Wo)*p);
                       del = acos( line );
                       phi = atan( (yg - yov) / (xg - xov) );
                       tho = del + phi - pi/2;
                       xo = xov - obj.R(c, Wo) * sin(tho);
                       yo = yov + obj.R(c, Wo) * cos(tho);
                       alp = atan( (yg - yo) / (xg - xo) );
                       if( xo > xg || yo > xg || ~isreal(alp) || ~isreal(alp) || tho < 0 || tho > pi/2 || isempty(alp) || isnan(alp))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
               end
               pivx = xg - obj.dim(c, 5) *sin(alp) + sign * lb * cos(alp) /2;
               pivy = yg + obj.dim(c, 5) *cos(alp) + sign * lb * sin(alp) /2;
               switch sblk(3)
                   case 1
                       y3v = blk(3) * st.riser;
                       y3  = y3v + obj.R(c, 3);
                       dly = abs(pivy - y3);
                       phi = asin(dly / rr);
                       gam = asin(lo / rr);
                       bet = gam + phi - pi/2;
                       x3  = pivx - lo * cos(bet) + obj.dim(c, 5) * sin(bet);
                       x3v = x3;
                       th3 = 0;
                       if (~isreal(bet) || isempty(bet) || isnan(bet))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
                   case 2
                       x3v = (blk(3) + 1) * st.tread;
                       x3  = x3v - obj.R(c, 3);
                       dlx = abs(pivx - x3);
                       phi = acos(dlx / rr);
                       gam = asin(lo / rr);
                       bet = gam + phi - pi/2;
                       y3 = pivy - lo * sin(bet) - obj.dim(c, 5) * cos(bet);
                       y3v = y3;
                       th3 = pi/2;
                       if (~isreal(bet) || isempty(bet) || isnan(bet))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
                   case 3
                       y3v = (blk(3) + 1) * st.riser;
                       x3v = (blk(3) + 1) * st.tread; 
                       pe = sqrt( (pivx - x3v)^2 + (pivy - y3v)^2 );
                       line = (obj.R(c, 3)^2 + pe^2 - rr^2) / (2*obj.R(c, 3)*pe);
                       del = acos( line );
                       phi = atan( (pivy - y3v) / (pivx - x3v) );
                       th3 = del + phi - pi/2;
                       x3 = x3v - obj.R(c, 3) * sin(th3);
                       y3 = y3v + obj.R(c, 3) * cos(th3);
                       phi2 = atan( (pivy - y3) / (pivx - x3));
                       gam = asin(lo / rr);
                       bet = gam + phi2 - pi/2;
                       if (~isreal(bet) || x3 > xo || y3 > yo || th3 > pi/2 || th3 < 0 || isempty(bet) || isnan(bet))
                           obj.val(c, i) = 0;
                           obj.tch(c, i) = 0;
                           return;
                       end
               end
           end
           obj.x(c, i, Wg) = xg;
           obj.x(c, i, Wo) = xo;
           obj.x(c, i,  3) = x3;
           obj.y(c, i, Wg) = yg;
           obj.y(c, i, Wo) = yo;
           obj.y(c, i,  3) = y3;
           obj.th(c, i,Wg) = ts;
           obj.th(c, i,Wo) = tho;
           obj.th(c, i, 3) = th3;
           obj.alpha(c, i)  = alp;
           obj.beta(c, i)   = bet;
           obj.xcm(c, i) = x3 + obj.dim(c, 1)*cos(bet) - (obj.dim(c, 5) + obj.dim(c, 6))*sin(bet);
           obj.ycm(c, i) = y3 + obj.dim(c, 1)*sin(bet) + (obj.dim(c, 5) + obj.dim(c, 6))*cos(bet);
           obj.val(c, i) = 1;
           xv(Wg) = xs;
           xv(Wo) = xov;
           xv( 3) = x3v;
           yv(Wg) = ys;
           yv(Wo) = yov;
           yv( 3) = y3v;
           obj.tch(c, i) = tcheck(obj, st, blk, sblk, xv, yv);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method tcheck
% Purpose: Performs a  total check on the validity of a generated solution.
%          Total check invovles checking both other wheel domains to ensure
%          they are on an appropriate location on a given set of stairs.
%          This is the method that drives the value of the tch class
%          property.
% Parameters:
%       obj -- the object for whom the check will be performed.
%       stair -- the stair whose domain is used for the check.
%       blk -- a 1D array of three elements denoting the block number for
%            each wheel (see the definitions in main.m for more information
%            on block numbers).
%       sblk -- a 1D array of three elements denoting the sub block number 
%              for each wheel (see the definitions in main.m for more 
%              information on sub block numbers).
%       x -- an array of the x coordinates of contact points of all wheels.
%       y -- an array of the y coordinates of contact points of all wheels.
% Returns:
%       1 -- if the wheels are located on the stairs.
%       0 -- if the wheels are not located on the stairs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Out = tcheck(obj, stair, blk, sblk, x, y)
            if(sblk(1) == 4)
                mch = stair.isOnDomain(x(2), y(2), blk(2), sblk(2), obj.R(2));
                rch = stair.isOnDomain(x(3), y(3), blk(3), sblk(3), obj.R(3));
                Out = rch && mch;
            elseif(sblk(2) == 4)
                fch = stair.isOnDomain(x(1), y(2), blk(1), sblk(1), obj.R(1));
                rch = stair.isOnDomain(x(2), y(3), blk(3), sblk(3), obj.R(3));
                Out = rch && fch;
            elseif(sub(3) == 4)
                fch = stair.isOnDomain(x(1), y(2), blk(1), sblk(1), obj.R(1));
                mch = stair.isOnDomain(x(2), y(2), blk(2), sblk(2), obj.R(2));
                Out = fch && mch;
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method draw
% Purpose: Creates a drawing of the rover as the plot. Takes into account
%          the current coordinates and orientation of each link.
% Parameters:
%       obj -- the object to be drawn.
%       c -- the configuration of the object to be drawn.
%       i -- the iteration of the object to be drawn.
% Returns: NONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj, c, i)
            p1x = obj.x(c, i, 3) - obj.dim(c, 5) * sin(obj.beta(c, i));
            p1y = obj.y(c, i, 3) + obj.dim(c, 5) * cos(obj.beta(c, i));
            p2x = p1x + (obj.dim(c, 1) + obj.dim(c, 2)) * cos(obj.beta(c, i));
            p2y = p1y + (obj.dim(c, 1) + obj.dim(c, 2)) * sin(obj.beta(c, i));
            p3x = obj.x(c, i, 2) - obj.dim(c, 5) * sin(obj.alpha(c, i));
            p3y = obj.y(c, i, 2) + obj.dim(c, 5) * cos(obj.alpha(c, i));
            p4x = obj.x(c, i, 1) - obj.dim(c, 5) * sin(obj.alpha(c, i));
            p4y = obj.y(c, i, 1) + obj.dim(c, 5) * cos(obj.alpha(c, i));
            pp1x = [obj.x(c, i,3), p1x];
            pp1y = [obj.y(c, i,3), p1y];
            pp3x = [obj.x(c, i,2), p3x];
            pp3y = [obj.y(c, i,2), p3y];
            pp5x = [obj.x(c, i,1), p4x];
            pp5y = [obj.y(c, i,1), p4y];
            pp2x = [p1x, p2x];
            pp2y = [p1y, p2y];
            pp4x = [p3x, p4x];
            pp4y = [p3y, p4y];
            circle(obj.x(c, i,1),obj.y(c, i,1),obj.R(c, 1));
            circle(obj.x(c, i,2),obj.y(c, i,2),obj.R(c, 2));
            circle(obj.x(c, i,3),obj.y(c, i,3),obj.R(c, 3));
            plot(pp1x, pp1y);
            plot(pp2x, pp2y);
            plot(pp3x, pp3y);
            plot(pp4x, pp4y);
            plot(pp5x, pp5y);
            circle(obj.xcm(c, i),obj.ycm(c, i),25);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method Optimize
% Purpose: Based on input dimensional limits, a defined number of
%          configurations are created and a trial is run per configuration.
%          During each trial the vertical center of gravity difference is
%          calculated and the best and worse configurations are stored in
%          the min and max class properties, respectively. 
%          NOTE 1: This method will be moved to the rover class in a
%                  subsequent update and will be implemented in a more
%                  generalized way.
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
        function obj = Optimize(obj, Wnum, st, Lrb, Llb, lrb, llb, db, hb, Rb, in, res)
            %Currently, the function does not optimise for wheel size,
            %although it can easily be adjusted to do so, but the
            %assumption is that we are currently buying wheels and it is
            %outside of our control. Rb is thus just a 3 item array with
            %each wheel radius. 
            it(1) = (Lrb(2) - Lrb(1)) / in;
            it(2) = (Llb(2) - Llb(1)) / in;
            it(3) = (lrb(2) - lrb(1)) / in;
            it(4) = (llb(2) - llb(1)) / in;
            it(5) = (db(2) - db(1)) / in;
            it(6) = (hb(2) - hb(1)) / in;
            c = 1;
            max = 0;
            min = 999999999999999999;
            for j1 = Lrb(1):it(1):Lrb(2)
                for j2 = Llb(1):it(2):Llb
                   for j3 = lrb(1):it(3):lrb(2)
                       for j4 = llb(1):it(4):llb(2)
                           for j5 = db(1):it(5):db(2)
                               for j6 = hb(1):it(6):hb(2)
                                   obj = obj.AssignGeometry(j2, j1, j4, j3, j5, j6, Rb, c);
                                   disp(c);
                                   if( (j4 + j3) < (Rb(1) + Rb(2)) )
                                       disp('The following iteration is skipped due to condition 1');
                                       c = c + 1;
                                       continue;
                                   elseif( j2 + j1 - Rb(3) < j4 + Rb(2) )
                                       disp('The following iteration is skipped due to condition 2');
                                       c = c + 1;
                                       continue;
                                   elseif( Rb(3) + j2 + j1 + j3 + Rb(1) > obj.lmax)
                                       disp('The following iteration is skipped due to condition 3');
                                       c = c + 1;
                                       continue;
                                   end
                                   obj = obj.Drive(st, Wnum, 1000, res, c);
                                   if( obj.warn(c) == 1)
                                       disp("WARNING");
                                   end
                                   if( obj.difyt(c) > max)
                                       obj.max = c;
                                       max = obj.difyt(c);
                                   elseif( obj.difyt(c) < min)
                                       obj.min = c;
                                       min = obj.difyt(c);
                                   end
                                   c = c + 1;
                               end
                           end
                       end
                   end
                end
            end
        end
    end
end