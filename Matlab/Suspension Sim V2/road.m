%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Road class
% Author: Maxim Kaller
% Purpose: Abstract class to create terrain for rover to drive on.
%          Contains generic properties that are useful for driving in
%          general regardless of the terrain profile.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef (Abstract) road
    properties
        g %gravity constant
    end
end