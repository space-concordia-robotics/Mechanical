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
            y = (obj.riser / obj.tread) * floor(x / obj.tread);
            plot(x, y);
        end
    end    
end