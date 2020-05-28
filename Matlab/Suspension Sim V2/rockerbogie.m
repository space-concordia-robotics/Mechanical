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
        function obj = rockerbogie(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height, Rf, Rm, Rr)
            obj.Ll    = rocker_left;
            obj.Lr    = rocker_right;
            obj.ll    = bogie_left;
            obj.lr    = bogie_right;
            obj.delta = bogie_height;
            obj.hc    = cg_height;
            obj.Rf    = Rf;
            obj.Rm    = Rm;
            obj.Rr    = Rr;
        end
        
        function F = Staireq(obj, stair, b, bb1, bb2, bb3, xi, yi, thi)
            %b (i) - step count of wheel i
            %bbi - portion of step of wheel i
            %bbi = 1 -> flat ground 
            %bbi = 2 -> step
            %bbi = 3 -> corner
            %bbi = 4 -> input
            lc = obj.Ll + obj.Lr;
            lb = 0.5 * (obj.ll + obj.lr);
            inpt = 0;
            syms prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
            eq1 = prx - obj.Rr * sin(thr) - obj.delta * sin(bet) + lc * cos(bet) == ppx;
            eq2 = pry + obj.Rr * cos(thr) + obj.delta * cos(bet) + lc * sin(bet) == ppy;
            eq3 = pmx - obj.Rm * sin(thm) - obj.delta * sin(alp) + lb * cos(alp) == ppx;
            eq4 = pmy + obj.Rm * cos(thm) + obj.delta * cos(alp) + lb * sin(alp) == ppy;
            eq5 = pfx - obj.Rf * sin(thf) + obj.delta * sin(alp) - lb * cos(alp) == ppx;
            eq6 = pfy + obj.Rf * cos(thf) + obj.delta * cos(alp) - lb * sin(alp) == ppy;
            switch bb1
                case 1
                    eq7  = b(1) * stair.riser       == pfy;
                    eq8  = thf == 0;
                case 2
                    eq7  = (b(1) + 1) * stair.tread == pfx;
                    eq8  = thf == pi/2;
                case 3
                    eq7  = (stair.slope) * pfx      == pfy;
                    eq8  = (b(1) + 1)*stair.riser   == pfy;
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
                        eq9  = (stair.slope) * pmx      == pmy;
                        eq10 = (b(2) + 1)*stair.riser   == pmy;
                    else
                        eq7  = (stair.slope) * pmx      == pmy;
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
                    eq9   = (stair.slope) * prx      == pry;
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
        
        function obj = configurePos(obj, Wheelnum, stair, x, y, th)
            if( stair.isOnstairs(x, y) == 0)
                disp('ERROR: configurePos input not on stair');
            else
                DetectPos(obj, Wheelnum, stair, x, y, th);
            end
        end
        
        function obj = DetectPos(obj, Wheelnum, stair, x, y, th)         
            bnum = stair.detectBlock(x);
            iblk = 4;
            solfound = 0;
                switch Wheelnum
                    case 1 %Front wheel contact pt given
                        for mid = bnum:-1:0
                            if (solfound == 1)
                                        break;
                            end
                            for midsub = 1:3
                                if (solfound == 1)
                                        break;
                                end
                                for rear = bnum:-1:0
                                    if (solfound == 1)
                                        break;
                                    end
                                    for rearsub = 1:3
                                        blk = [bnum, mid, rear];
                                        sb = [iblk, midsub, rearsub];
                                        obj = SolveStEq(obj, stair, blk, sb, x, y, th);
                                        if(obj.tch == 1)
                                            solfound = 1;
                                            break;
                                        end
                                    end
                                end
                            end
                        end
                    case 2 %Mid   wheel contact pt given
                        for fr = bnum:1000000 %Arbitrarily large number
                            if (solfound == 1)
                                        break;
                            end
                            for frsub = 1:3
                                if (solfound == 1)
                                        break;
                                end
                                for rear = bnum:-1:0
                                    if (solfound == 1)
                                        break;
                                    end
                                    for rearsub = 1:3
                                        blk = [fr, bnum, rear];
                                        sb = [frsub, iblk, rearsub];
                                        cnd = SolveStEq(obj, stair, blk, sb, x, y, th);
                                        if(cnd == 1)
                                            solfound = 1;
                                            break;
                                        end
                                    end
                                end
                            end
                        end
                    case 3 %Rear  wheel contact pt given
                        for fr = bnum:1000000 %Arbitrarily large number
                            if (solfound == 1)
                                        break;
                            end
                            for frsub = 1:3
                                if (solfound == 1)
                                        break;
                                end
                                for mid = bnum:1000000 
                                    if (solfound == 1)
                                        break;
                                    end
                                    for midsub = 1:3
                                        blk = [fr, mid, bnum];
                                        sb = [frsub, midsub, iblk];
                                        cnd = SolveStEq(obj, stair, blk, sb, x, y, th);
                                        if(cnd == 1)
                                            solfound = 1;
                                            break;
                                        end
                                    end
                                end
                            end
                        end
                    otherwise
                    disp('ERROR Invalid Parameters for DetectPos');
                end
        end
        
        function obj = SolveStEq(obj, stair, blck, sub, x, y, th)
            obj.tch = 0;
            sol = obj.Staireq(stair, blck, sub(1), sub(2), sub(3), x, y, th);
            if(isempty(sol.prx) || isempty(sol.pmx) || isempty(sol.pfx))
                disp('ERROR: vpasolve error. Empty values.');
            else
                %prx pry pmx pmy pfx pfy thr thm thf alp bet ppx ppy
                if(sub(1) == 4)
                    mch = stair.isOnDomain(sol.pmy, sol.pmy, blck(2), sub(2));
                    rch = stair.isOnDomain(sol.prx, sol.pry, blck(3), sub(3));
                    obj.tch = rch && mch;
                elseif(sub(2) == 4)
                    fch = stair.isOnDomain(sol.pfx, sol.pfy, blck(1), sub(1));
                    rch = stair.isOnDomain(sol.prx, sol.pry, blck(3), sub(3));
                    obj.tch = rch && fch;
                elseif(sub(3) == 4)
                    fch = stair.isOnDomain(sol.pfx, sol.pfy, blck(1), sub(1));
                    mch = stair.isOnDomain(sol.pmy, sol.pmy, blck(2), sub(2));
                    obj.tch = fch && mch;
                end
                obj.alpha = sol.alp;
                obj.beta = sol.bet;
                obj.xr = sol.prx - obj.Rr * sin(sol.thr);
                obj.yr = sol.pry + obj.Rr * cos(sol.thr);
                obj.xm = sol.pmx - obj.Rm * sin(sol.thm);
                obj.ym = sol.pmy + obj.Rm * cos(sol.thm);
                obj.xf = sol.pfx - obj.Rf * sin(sol.thf);
                obj.yf = sol.pfy + obj.Rf * cos(sol.thf);
                obj.xcm = obj.Lr*cos(sol.bet) - (obj.delta + obj.hc)*sin(sol.bet);
                obj.ycm = obj.Lr*sin(sol.bet) + (obj.delta + obj.hc)*cos(sol.bet);
            end
        end
        
        function draw(obj)
            circle(obj.xf,obj.yf,obj.Rf);
            circle(obj.xm,obj.ym,obj.Rm);
            circle(obj.xr,obj.yr,obj.Rr);
        end
    end
end