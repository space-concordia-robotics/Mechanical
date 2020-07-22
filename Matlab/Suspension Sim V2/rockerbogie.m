classdef rockerbogie < rover
    properties
        Ll
        Lr
        ll
        lr
        delta
        hc
        alpha
        beta
    end
    methods
        function obj = rockerbogie(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Ra)
            obj.Ll(1)    = rocker_left;
            obj.Lr(1)    = rocker_right;
            obj.ll(1)    = bogie_left;
            obj.lr(1)    = bogie_right;
            obj.delta(1) = bogie_height;
            obj.hc(1)    = cg_height;
            obj.R(1,1)  = Ra(1);
            obj.R(1,2)  = Ra(2);
            obj.R(1,3)  = Ra(3);
            obj.ddyt  = 0;
            obj.difyt = 0;
        end
        
        function obj = AssignGeometry(obj, rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Ra, c)
            %Defines geometric properties only
            obj.Ll(c)    = rocker_left;
            obj.Lr(c)    = rocker_right;
            obj.ll(c)    = bogie_left;
            obj.lr(c)    = bogie_right;
            obj.delta(c) = bogie_height;
            obj.hc(c)    = cg_height;
            obj.R(c,1)  = Ra(1);
            obj.R(c,2)  = Ra(2);
            obj.R(c,3)  = Ra(3);
        end
        
        function F = Staireq(obj, stair, b, bb1, bb2, bb3, xi, yi, thi, c)
            %b (i) - step count of wheel i
            %bbi - portion of step of wheel i
            %bbi = 1 -> flat ground 
            %bbi = 2 -> step
            %bbi = 3 -> corner
            %bbi = 4 -> input
            lc = obj.Ll(c) + obj.Lr(c);
            inpt = 0;
            syms prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
            eq1 = prx - obj.R(c,3) * sin(thr) - obj.delta(c) * sin(bet) +     lc * cos(bet) == ppx;
            eq2 = pry + obj.R(c,3) * cos(thr) + obj.delta(c) * cos(bet) +     lc * sin(bet) == ppy;
            eq3 = pmx - obj.R(c,2) * sin(thm) - obj.delta(c) * sin(alp) + obj.ll(c) * cos(alp) == ppx;
            eq4 = pmy + obj.R(c,2) * cos(thm) + obj.delta(c) * cos(alp) + obj.ll(c) * sin(alp) == ppy;
            eq5 = pfx - obj.R(c,1) * sin(thf) - obj.delta(c) * sin(alp) - obj.lr(c) * cos(alp) == ppx;
            eq6 = pfy + obj.R(c,1) * cos(thf) + obj.delta(c) * cos(alp) - obj.lr(c) * sin(alp) == ppy;
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
        
        function obj = DetectPos(obj, Wheelnum, stair, x, y, th, c,  i)
            if( stair.isOnstairs(x, y) == 0)
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
                    disp('Error is Distloop. Incorrect Wheelnum');
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
                                obj = SolveStEq(obj, stair, blk, sb, x, y, th, c, i);
                            end
                            if(obj.tch(c, i) == 1 && obj.val(c, i) == 1)
                                solfound = 1;
                                if(sb(1) == 2 && sb(2) == 2)
                                    obj.warn(c) = 1;
                                end
                                %disp('SOLUTION INFO');
                                %disp(blk);
                                %disp(sb);
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
        
        function obj = SolveStEq(obj, stair, blck, sub, x, y, th, c, i)
            obj.tch = 0;
            sol = obj.Staireq(stair, blck, sub(1), sub(2), sub(3), x, y, th, c);
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
        
        function obj = cForm(obj, st, Wg, xs, ys, ts, blk, sblk, c, i)
           xg = xs - obj.R(c, Wg)*sin(ts);
           yg = ys + obj.R(c, Wg)*cos(ts);
           lb = obj.ll(c) + obj.lr(c);
           lo = obj.Ll(c) + obj.Lr(c);
           rr = sqrt ( (obj.delta(c))^2 + lo^2);
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
               pivx = xg - obj.delta(c) *sin(alp) + sign * lb * cos(alp) /2;
               pivy = yg + obj.delta(c) *cos(alp) + sign * lb * sin(alp) /2;
               switch sblk(3)
                   case 1
                       y3v = blk(3) * st.riser;
                       y3  = y3v + obj.R(c, 3);
                       dly = abs(pivy - y3);
                       phi = asin(dly / rr);
                       gam = asin(lo / rr);
                       bet = gam + phi - pi/2;
                       x3  = pivx - lo * cos(bet) + obj.delta(c) * sin(bet);
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
                       y3 = pivy - lo * sin(bet) - obj.delta(c) * cos(bet);
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
           obj.xcm(c, i) = x3 + obj.Ll(c)*cos(bet) - (obj.delta(c) + obj.hc(c))*sin(bet);
           obj.ycm(c, i) = y3 + obj.Ll(c)*sin(bet) + (obj.delta(c) + obj.hc(c))*cos(bet);
           obj.val(c, i) = 1;
           xv(Wg) = xs;
           xv(Wo) = xov;
           xv( 3) = x3v;
           yv(Wg) = ys;
           yv(Wo) = yov;
           yv( 3) = y3v;
           obj.tch(c, i) = tcheck(obj, st, blk, sblk, xv, yv);
        end
        
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
           
        function draw(obj, c, i)
            %Creates a drawing of the rover as the plot. Takes into account
            %the current orientation of each link.
            p1x = obj.x(c, i, 3) - obj.delta(c) * sin(obj.beta(c, i));
            p1y = obj.y(c, i, 3) + obj.delta(c) * cos(obj.beta(c, i));
            p2x = p1x + (obj.Ll(c) + obj.Lr(c)) * cos(obj.beta(c, i));
            p2y = p1y + (obj.Ll(c) + obj.Lr(c)) * sin(obj.beta(c, i));
            p3x = obj.x(c, i,2) - obj.delta(c) * sin(obj.alpha(c, i));
            p3y = obj.y(c, i,2) + obj.delta(c) * cos(obj.alpha(c, i));
            p4x = obj.x(c, i,1) - obj.delta(c) * sin(obj.alpha(c, i));
            p4y = obj.y(c, i,1) + obj.delta(c) * cos(obj.alpha(c, i));
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
        
        function obj = Optimize(obj, Wnum, st, Lrb, Llb, lrb, llb, db, hb, Rb, in, res)
            %Currently, the function does not optimise for wheel size,
            %although it can easily be adjusted to do so, but the
            %assumption is that we are currently buying wheels and it is
            %outside of our control. Rb is thus just a 3 item array with
            %each wheel radius. Each other argument is an array of two with
            %the min and max optimization ranges.
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
                                   elseif( Rb(3) + j2 + j1 + j3 + Rb(1) > 1200)
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