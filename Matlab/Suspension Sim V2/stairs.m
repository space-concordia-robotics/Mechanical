classdef stairs < road
    properties
        tread
        riser
        slope
    end
    methods
        function obj = stairs(t, r, grav)
            obj.g = grav;
            obj.tread = t;
            obj.riser = r;
            if (obj.tread ~= 0)
                obj.slope = atan(obj.riser / obj.tread);
            end
        end
        
        function output = isonriser(obj, x)
            if (mod(x,obj.tread) == 0) && (x ~= 0)
                output = 1;
            else
                output = 0;
            end
        end
        
        function draw_stairs(obj, length) %draws stairs from x = 0 to x = length
            x = linspace(0, length, 10000);
            y = (obj.riser ) * floor(x / obj.tread);
            plot(x, y);
        end
        
        function output = determine_obs_height(obj, x, y) 
            y_stair = obj.riser * floor(x / obj.tread);
            output = y - y_stair; %NOTE: Negative output cannot be provided
        end
        
        function output = getLastStep(obj, x) 
            %if currently on step, will take the previous
            %outputs 0 if no steps yet climbed
            if (isonriser(obj, x) == 1)
                output = (floor(x / obj.tread) - 1) * obj.tread;
            else
                output = floor(x / obj.tread) * obj.tread;
            end
        end
        
        function output = isOnstairs(obj, x, y)
            if (isonriser(obj, x) == 1)
                high_end = (x / obj.tread) * obj.riser;
                low_end = high_end - obj.riser;
                if(y >= high_end || y <= low_end)
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
        
        function [xmin, xmax, ymin, ymax] = detectDomain(obj, bnum)
            %the first block is characterized by 0
            %a block is switched right after the step
            %so xmin is not included in the domain
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
        
        function Output = isOnDomain(obj, x, y, bnum, sbnum, R)
            [xmin, xmax, ymin, ymax] = obj.detectDomain(bnum);
            if (sbnum == 1) 
                if (((x > (xmax - R)) || (x < xmin)))
                    Output = 0;
                else
                    Output = 1;
                end
            elseif (sbnum == 2)
                if ((y > ymax) || (y < ymin))
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
        
    end    
end