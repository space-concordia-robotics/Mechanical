classdef (Abstract) rover
    properties
        dim
        dimnum
        R
        Mp
        Mw
        Lmax
        xcm
        xcmo
        ycm
        ycmo
        x
        y
        th
        tch
        val
        ddy
        ddyt
        dify
        difyt
        warn
        max
        min
        its
    end
    
    methods (Abstract)
        obj = DetectPos(obj, Wheelnum, stair, x, y, th, c, i);
        obj = Optimize(obj, Wnum, st, Lrb, Llb, lrb, llb, db, hb, Rb);
    end
    
    methods
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
        
        function Animate(obj, c, maxx)
            for i = 1:obj.its
               hold on;
               st.draw_stairs(2 * maxx);
               obj.draw(c, i);
               hold off;
               pause(0.04);
            end
        end
        
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