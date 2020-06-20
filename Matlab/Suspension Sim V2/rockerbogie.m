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
            obj.R(1)    = Ra(1);
            obj.R(2)    = Ra(2);
            obj.R(3)    = Ra(3);
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
        
        function obj = DetectPos(obj, Wheelnum, stair, x, y, th)
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
                            if (obj.isStepPossible(stair, Wheelnum, x, y, th, blk, sb) == 0)
                                continue;
                            end
                            obj = SolveStEq(obj, stair, blk, sb, x, y, th);
                            if(obj.tch == 1)
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
                obj.xf = [];
            else
                %prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
                if(sub(1) == 4)
                    mch = stair.isOnDomain(sol.pmx, sol.pmy, blck(2), sub(2), obj.R(2));
                    rch = stair.isOnDomain(sol.prx, sol.pry, blck(3), sub(3), obj.R(3));
                    obj.tch = rch && mch;
                elseif(sub(2) == 4)
                    fch = stair.isOnDomain(sol.pfx, sol.pfy, blck(1), sub(1), obj.R(1));
                    rch = stair.isOnDomain(sol.prx, sol.pry, blck(3), sub(3), obj.R(3));
                    obj.tch = rch && fch;
                elseif(sub(3) == 4)
                    fch = stair.isOnDomain(sol.pfx, sol.pfy, blck(1), sub(1), obj.R(1));
                    mch = stair.isOnDomain(sol.pmy, sol.pmy, blck(2), sub(2), obj.R(2));
                    obj.tch = fch && mch;
                end
                obj.alpha = sol.alp;
                obj.beta = sol.bet;
                obj.xr = sol.prx - obj.R(3) * sin(sol.thr);
                obj.yr = sol.pry + obj.R(3) * cos(sol.thr);
                obj.xm = sol.pmx - obj.R(2) * sin(sol.thm);
                obj.ym = sol.pmy + obj.R(2) * cos(sol.thm);
                obj.xf = sol.pfx - obj.R(1) * sin(sol.thf);
                obj.yf = sol.pfy + obj.R(1) * cos(sol.thf);
                obj.xcm = sol.prx + obj.Ll*cos(sol.bet) - (obj.delta + obj.hc)*sin(sol.bet);
                obj.ycm = sol.pry + obj.Ll*sin(sol.bet) + (obj.delta + obj.hc)*cos(sol.bet);
                obj.th(1) = sol.thf;
                obj.th(2) = sol.thm;
                obj.th(3) = sol.thr;
            end
        end
        
        function Out = isStepPossible(obj, st, Wg, xs, ys, ts, blk, sblk)
           %The aim of the function is to check if a solution is possible.
           %It does soby checking and performing four comparisons:
           %C1 -> Compare front and mid wheel distnace
           %C2 -> Compare front and rear wheel distance
           %C3 -> Compare mid and rear wheel distance
           %C4 -> Compare rear wheel and pivot point distance
           %Each variable checks if a comparison can be done.
           %If it cannot, it will automatically pass the comparison.
           %For the function to turn positive, all comparisons must be
           %positive.
           largenum = 1000000000; %approximate infinite distance
           tol = 0.001; %tolerance for float comparisons
           xg = xs - obj.R(Wg)*sin(ts);
           yg = ys + obj.R(Wg)*cos(ts);
           lb = obj.ll + obj.lr;
           lo = obj.Ll + obj.Lr;
           dmax(1) = obj.ll + obj.lr + tol;
           dmin(1) = dmax(1) - 2*tol;
%            dmax(2) = (obj.Ll + obj.Lr);
%            dmin(2) = (obj.Ll + obj.Lr) - obj.lr;
%            dmax(3) = dmax(2);
%            dmin(3) = (obj.Ll + obj.Lr) - obj.ll;
           dmax(2) = sqrt(lo^2 + obj.delta^2) + tol;
           dmin(2) = dmax(2) - 2*tol;
           %Caclculate first distance for comparison
           if (Wg ~= 3)
               sign = (-1)^Wg;
               Wo = 3 - Wg;
               switch sblk(Wo)
                   case 1
                       loopnum = 2;
                       yo  = blk(Wo) * st.riser + obj.R(Wo);
                       alp = asin( (yg - yo)/ lb );
                       pmin(1) = abs(yg - yo);
                       pmax(1) = largenum;
                   case 2
                       loopnum = 2;
                       xo  = (blk(Wo) + 1) * st.tread - obj.R(Wo);
                       alp = acos( (xg - xo)/ lb );
                       pmin(1) = abs(xg - xo);
                       pmax(1) = largenum;
                   case 3 
                       loopnum = 2;
                       y3 = (blk(Wo) + 1) * st.riser;
                       x3 = (blk(Wo) + 1) * st.tread; 
                       p = sqrt( (xg - x3)^2 + (yg - y3)^2 );
                       del = acos( (lb^2 - obj.R(Wo)^2 - p^2) / (2*obj.R(Wo)*p) );
                       bet = atan( (yg - y3) / (xg - x3) );
                       th = del + bet - pi/2;
                       xo = x3 - obj.R(Wo) * sin(th);
                       yo = x3 + obj.R(Wo) * cos(th);
                       if( xo > xg || yo > xg || p^2 + obj.R(Wo) < lb^2)
                           Out = 0;
                           return;
                       end
                       pmin(1) = th;
                       pmax(1) = th;
                       dmin(1) = 0;
                       dmax(1) = pi/2;
                       alp = asin( (p/lb) * sin(del) ) + th - pi/2;
               end
               pivx = xg - obj.delta*sin(alp) + sign * lb * cos(alp) /2;
               pivy = yg + obj.delta*cos(alp) + sign * lb * sin(alp) /2;
               switch sblk(3)
                   case 1
                       y3 = blk(3) * st.riser + obj.R(3);
                       pmin(2) = abs(pivy - y3);
                       pmax(2) = largenum;
                   case 2
                       x3 = (blk(3) + 1) * st.tread - obj.R(3);
                       pmin(2) = abs(pivx - x3);
                       pmax(2) = largenum;
                   case 3
                       y3min = (blk(3) + 1) * st.riser + obj.R(3);
                       x3min = (blk(3) + 1) * st.tread; 
                       y3max = (blk(3) + 1) * st.riser;
                       x3max = (blk(3) + 1) * st.tread - obj.R(3);
                       pmin(2) = sqrt((y3min - pivy)^2 + (x3min - pivx)^2);  
                       pmax(2) = sqrt((y3max - pivy)^2 + (x3max - pivx)^2);
               end
           end
           for i = 1:loopnum
               if (pmin(i) > dmax(i) || pmax(i) < dmin(i))
                   Out = 0;
                   return;
               end
           end
           Out = 1;
%            switch sblk
%                case 1
%                    pmin = abs(blk * st.riser + obj.R(Wi) - yg);
%                    pmax = largenum;
%                case 2
%                    pmin = abs((blk + 1) * st.tread - obj.R(Wi) - xg);
%                    pmax = largenum;
%                case 3
%                    ymin = (blk + 1) * st.riser + obj.R(Wi);
%                    xmin = (blk + 1) * st.tread;
%                    pmin = sqrt((ymin - yg)^2 + (xmin - xg)^2);    
%                    ymax = (blk + 1) * st.riser;
%                    xmax = (blk + 1) * st.tread - obj.R(Wi);
%                    pmax = sqrt((ymax - yg)^2 + (xmax - xg)^2);
%                otherwise 
%                    disp('sblk assignment issue');
%            end
%            if (pmin > maxDist || pmax < minDist)
%                Out = 0;
%                disp('Distance case called, for block:');
%                disp(blk);
%                disp(sblk);
%                disp("pmin pmax maxDist minDist");
%                disp(pmin);
%                disp(pmax);
%                disp(maxDist);
%                disp(minDist);
%            else
%                Out = 1;
%            end
        end
        
        function draw(obj)
            p1x = obj.xr - obj.delta * sin(obj.beta);
            p1y = obj.yr + obj.delta * cos(obj.beta);
            p2x = p1x + (obj.Ll + obj.Lr) * cos(obj.beta);
            p2y = p1y + (obj.Ll + obj.Lr) * sin(obj.beta);
            p3x = obj.xm - obj.delta * sin(obj.alpha);
            p3y = obj.ym + obj.delta * cos(obj.alpha);
            p4x = obj.xf - obj.delta * sin(obj.alpha);
            p4y = obj.yf + obj.delta * cos(obj.alpha);
            pp1x = [obj.xr, p1x];
            pp1y = [obj.yr, p1y];
            pp2x = [p1x, p2x];
            pp2y = [p1y, p2y];
            pp3x = [obj.xm, p3x];
            pp3y = [obj.ym, p3y];
            pp4x = [p3x, p4x];
            pp4y = [p3y, p4y];
            pp5x = [obj.xf, p4x];
            pp5y = [obj.yf, p4y];
            circle(obj.xf,obj.yf,obj.R(1));
            circle(obj.xm,obj.ym,obj.R(2));
            circle(obj.xr,obj.yr,obj.R(3));
            plot(pp1x, pp1y);
            plot(pp2x, pp2y);
            plot(pp3x, pp3y);
            plot(pp4x, pp4y);
            plot(pp5x, pp5y);
            circle(obj.xcm,obj.ycm,25);
        end
    end
end