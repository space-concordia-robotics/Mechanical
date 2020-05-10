classdef rockerbogie < rover
    properties
        Ll
        Lr
        ll
        lr
        delta
        hc
    end
    methods
        function obj = rockerbogie(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height)
            setGeometry(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height);
        end
        function obj = setGeometry(rocker_left, rocker_right, bogie_left, bogie_right, bogie_height, cg_height)
            obj.Ll = rocker_left;
            obj.Ll = rocker_right;
            obj.Ll = bogie_left;
            obj.Ll = bogie_right;
            obj.Ll = bogie_height;
            obj.Ll = cg_height;
        end
    end
end