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
            %Defines geometric properties only
            obj.Ll    = rocker_left;
            obj.Lr    = rocker_right;
            obj.ll    = bogie_left;
            obj.lr    = bogie_right;
            obj.delta = bogie_height;
            obj.hc    = cg_height;
            obj.R(1)  = Ra(1);
            obj.R(2)  = Ra(2);
            obj.R(3)  = Ra(3);
        end
        
        function F = Staireq(obj, stair, b, bb1, bb2, bb3, xi, yi, thi)
            %b (i) - step count of wheel i
            %bbi - portion of step of wheel i
            %bbi = 1 -> flat ground 
            %bbi = 2 -> step
            %bbi = 3 -> corner
            %bbi = 4 -> input
            lc = obj.Ll + obj.Lr;
            inpt = 0;
            syms prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
            eq1 = prx - obj.R(3) * sin(thr) - obj.delta * sin(bet) +     lc * cos(bet) == ppx;
            eq2 = pry + obj.R(3) * cos(thr) + obj.delta * cos(bet) +     lc * sin(bet) == ppy;
            eq3 = pmx - obj.R(2) * sin(thm) - obj.delta * sin(alp) + obj.ll * cos(alp) == ppx;
            eq4 = pmy + obj.R(2) * cos(thm) + obj.delta * cos(alp) + obj.ll * sin(alp) == ppy;
            eq5 = pfx - obj.R(1) * sin(thf) - obj.delta * sin(alp) - obj.lr * cos(alp) == ppx;
            eq6 = pfy + obj.R(1) * cos(thf) + obj.delta * cos(alp) - obj.lr * sin(alp) == ppy;
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
        
        function obj = DetectPos(obj, Wheelnum, stair, x, y, th, i)
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
                                obj = obj.cForm( stair, Wheelnum, x, y, th, blk, sb, i);
                            else
                                obj = SolveStEq(obj, stair, blk, sb, x, y, th);
                            end
                            if(obj.tch(i) == 1 && obj.val(i) == 1)
                                solfound = 1;
                                disp('SOLUTION INFO');
                                disp(blk);
                                disp(sb);
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
        
        function obj = SolveStEq(obj, stair, blck, sub, x, y, th)
            obj.tch = 0;
            sol = obj.Staireq(stair, blck, sub(1), sub(2), sub(3), x, y, th);
            if(isempty(sol.prx) || isempty(sol.pmx) || isempty(sol.pfx))
                disp('ERROR: vpasolve error. Empty values. Following inputs:');
                disp(blck);
                disp(sub);
                obj.x = [];
            else
                %prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
                obj.alpha = sol.alp;
                obj.beta = sol.bet;
                obj.x(3) = sol.prx - obj.R(3) * sin(sol.thr);
                obj.y(3) = sol.pry + obj.R(3) * cos(sol.thr);
                obj.x(2) = sol.pmx - obj.R(2) * sin(sol.thm);
                obj.y(2) = sol.pmy + obj.R(2) * cos(sol.thm);
                obj.x(1) = sol.pfx - obj.R(1) * sin(sol.thf);
                obj.y(1) = sol.pfy + obj.R(1) * cos(sol.thf);
                obj.tch = tcheck(obj, stair, blck, sub, obj.x, obj.y);
                obj.xcm = sol.prx + obj.Ll*cos(sol.bet) - (obj.delta + obj.hc)*sin(sol.bet);
                obj.ycm = sol.pry + obj.Ll*sin(sol.bet) + (obj.delta + obj.hc)*cos(sol.bet);
                obj.th(1) = sol.thf;
                obj.th(2) = sol.thm;
                obj.th(3) = sol.thr;
            end
        end
        
        function obj = cForm(obj, st, Wg, xs, ys, ts, blk, sblk, i)
           xg = xs - obj.R(Wg)*sin(ts);
           yg = ys + obj.R(Wg)*cos(ts);
           lb = obj.ll + obj.lr;
           lo = obj.Ll + obj.Lr;
           rr = sqrt ( (obj.delta)^2 + lo^2);
           if (Wg ~= 3)
               sign = (-1)^Wg;
               Wo = 3 - Wg;
               switch sblk(Wo)
                   case 1
                       yov = blk(Wo) * st.riser;
                       yo  = yov + obj.R(Wo);
                       alp = asin( (yg - yo)/ lb );
                       xo  = xg + sign * lb * cos(alp);
                       xov = xo;
                       tho = 0;
                       if( ~isreal(alp))
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
                   case 2
                       xov = (blk(Wo) + 1) * st.tread;
                       xo  = xov - obj.R(Wo);
                       alp = acos( (xg - xo)/ lb );
                       yo  = yg + sign * lb * sin(alp);
                       yov = yo;
                       tho = pi/2;
                       if( ~isreal(alp))
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
                   case 3 
                       yov = (blk(Wo) + 1) * st.riser;
                       xov = (blk(Wo) + 1) * st.tread; 
                       p = sqrt( (xg - xov)^2 + (yg - yov)^2 );
                       line = (obj.R(Wo)^2 + p^2 - lb^2) / (2*obj.R(Wo)*p);
                       del = acos( line );
                       phi = atan( (yg - yov) / (xg - xov) );
                       tho = del + phi - pi/2;
                       xo = xov - obj.R(Wo) * sin(tho);
                       yo = yov + obj.R(Wo) * cos(tho);
                       alp = asin( (p/lb) * sin(del) ) + tho - pi/2;
                       if( xo > xg || yo > xg || ~isreal(alp) || ~isreal(alp) || tho < 0 || tho > pi/2)
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
               end
               pivx = xg - obj.delta*sin(alp) + sign * lb * cos(alp) /2;
               pivy = yg + obj.delta*cos(alp) + sign * lb * sin(alp) /2;
               switch sblk(3)
                   case 1
                       y3v = blk(3) * st.riser;
                       y3  = y3v + obj.R(3);
                       dly = abs(pivy - y3);
                       phi = asin(dly / rr);
                       gam = asin(lo / rr);
                       bet = gam + phi - pi/2;
                       x3  = pivx - lo * cos(bet) + obj.delta * sin(bet);
                       x3v = x3;
                       th3 = 0;
                       if (~isreal(bet))
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
                   case 2
                       x3v = (blk(3) + 1) * st.tread;
                       x3  = x3v - obj.R(3);
                       dlx = abs(pivx - x3);
                       phi = acos(dlx / rr);
                       gam = asin(lo / rr);
                       bet = gam + phi - pi/2;
                       y3 = pivy - lo * sin(bet) - obj.delta * cos(bet);
                       y3v = y3;
                       th3 = pi/2;
                       if (~isreal(bet))
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
                   case 3
                       y3v = (blk(3) + 1) * st.riser;
                       x3v = (blk(3) + 1) * st.tread; 
                       pe = sqrt( (pivx - x3v)^2 + (pivy - y3v)^2 );
                       line = (obj.R(3)^2 + pe^2 - rr^2) / (2*obj.R(3)*pe);
                       del = acos( line );
                       phi = atan( (pivy - y3v) / (pivx - x3v) );
                       th3 = del + phi - pi/2;
                       x3 = x3v - obj.R(3) * sin(th3);
                       y3 = y3v + obj.R(3) * cos(th3);
                       phi2 = atan( (pivy - y3) / (pivx - x3));
                       gam = asin(lo / rr);
                       bet = gam + phi2 - pi/2;
                       if (~isreal(bet) || x3 > xo || y3 > yo || th3 > pi/2 || th3 < 0)
                           obj.val(i) = 0;
                           obj.tch(i) = 0;
                           return;
                       end
               end
           end
           obj.x(i, Wg) = xg;
           obj.x(i, Wo) = xo;
           obj.x(i,  3) = x3;
           obj.y(i, Wg) = yg;
           obj.y(i, Wo) = yo;
           obj.y(i,  3) = y3;
           obj.th(i,Wg) = ts;
           obj.th(i,Wo) = tho;
           obj.th(i, 3) = th3;
           obj.alpha(i)  = alp;
           obj.beta(i)   = bet;
           obj.xcm(i) = x3 + obj.Ll*cos(bet) - (obj.delta + obj.hc)*sin(bet);
           obj.ycm(i) = y3 + obj.Ll*sin(bet) + (obj.delta + obj.hc)*cos(bet);
           obj.val(i) = 1;
           xv(Wg) = xs;
           xv(Wo) = xov;
           xv( 3) = x3v;
           yv(Wg) = ys;
           yv(Wo) = yov;
           yv( 3) = y3v;
           obj.tch(i) = tcheck(obj, st, blk, sblk, xv, yv);
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
           
        function draw(obj, i)
            p1x = obj.x(i, 3) - obj.delta * sin(obj.beta(i));
            p1y = obj.y(i, 3) + obj.delta * cos(obj.beta(i));
            p2x = p1x + (obj.Ll + obj.Lr) * cos(obj.beta(i));
            p2y = p1y + (obj.Ll + obj.Lr) * sin(obj.beta(i));
            p3x = obj.x(i,2) - obj.delta * sin(obj.alpha(i));
            p3y = obj.y(i,2) + obj.delta * cos(obj.alpha(i));
            p4x = obj.x(i,1) - obj.delta * sin(obj.alpha(i));
            p4y = obj.y(i,1) + obj.delta * cos(obj.alpha(i));
            pp1x = [obj.x(i,3), p1x];
            pp1y = [obj.y(i,3), p1y];
            pp3x = [obj.x(i,2), p3x];
            pp3y = [obj.y(i,2), p3y];
            pp5x = [obj.x(i,1), p4x];
            pp5y = [obj.y(i,1), p4y];
            pp2x = [p1x, p2x];
            pp2y = [p1y, p2y];
            pp4x = [p3x, p4x];
            pp4y = [p3y, p4y];
            circle(obj.x(i,1),obj.y(i,1),obj.R(1));
            circle(obj.x(i,2),obj.y(i,2),obj.R(2));
            circle(obj.x(i,3),obj.y(i,3),obj.R(3));
            plot(pp1x, pp1y);
            plot(pp2x, pp2y);
            plot(pp3x, pp3y);
            plot(pp4x, pp4y);
            plot(pp5x, pp5y);
            circle(obj.xcm(i),obj.ycm(i),25);
        end
    end
end