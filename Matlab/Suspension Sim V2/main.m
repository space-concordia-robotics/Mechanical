%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              MARS ROVER QUASI STATIC GEOMETRY OPTIMIZATION   
%                              VERSION: 0.3.1
% Author: Maxim Kaller
% Description: 
% Units: 
%   ALL LENGTH UNITS -- milimeters (mm)
%   ALL ANGLE UNITS -- radians (rad)
%   ALL FORCE UNITS -- Newtons (N)
% Definitions:
%   Block - a block constitutes a set of consecutive tread, riser and 
%           corner portions of a stair. In this sense, a block is the main
%           building block of a stair and stairs can be considered as
%           chains of blocks starting one after the other (see Stair.PNG). 
%   Block Number - A number describing the chronological order of a given
%                  block. The first block is denoted by a block number of 0
%                  and every subsequent block has a block number that is an
%                  increment of the previous block number. 
%   Configuration - Commonly denoted with a letter c, it denotes a set of
%                   dimensions that correspond to set of instances in a
%                   trial.
%   Contact Angle - The third variable describing the road - wheel
%                   interface. This is the angle formed between the wheel's
%                   vertical axis and the axis containing the wheel's
%                   center point and the contact point between the wheel
%                   and the road (see wheel.PNG).
%   Conctact Coordinate - The x or y coordinate value for a contact point.
%   Contact Point - The point where a wheel makes contact with the road
%                   profile. This point is characterized by two contact
%                   coordinates (horizontal and vertical) and a contact
%                   angle.
%   Corner - The third stair sub block which denotes a portion of the 
%            road - wheel interface where only the contact angle varies. 
%            A corner has a contact angle between 0 to pi/2 radians (see 
%            Stair.PNG). 
%   Instance - A kinematic solution of a rover configuration with
%              coordinate input for a given wheel in contact with the road.
%              Usually instances are solved sequentially with the input
%              wheel increasing its location on the road as part of a
%              trial.
%   Riser - The vertical portion and the second sub block of a stair. When
%           a wheel is on a riser it is considered to be taking a step (see
%           Stair.PNG).
%   Sub Block - Used to differentiate between the three different profiles
%               in a stair block: the first is the tread which is denoted
%               by a constant horizontal contact coordinate and a contact
%               angle of zero. The second is the riser which is denoted by
%               a constant vertical contact coordinate and a contact angle
%               of pi/2. Lastly, the third sub block is the corner which
%               has constant vertical and horizontal contact coordinates,
%               but a varying contact angle between 0 and pi/2 rad (see
%               Stair.PNG).
%   Tread - The horizontal portion and the first sub block of a stair. Any
%           portion of the stair road prior to the first riser is
%           considered a tread (see Stair.PNG).
%   Trial - A set of instances that corresponds to driving through a road
%           under a single rover configuration.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TO DO (must be complete before work on next major release can begin):
% - Remove usage of ycmo and xcmo properties.
% - Create baseline configuration and use as a reference.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
a = stairs(240, 200, 9.8065);
R = [150, 150, 150];
dims = [481, 194, 225, 225, 170, 94];
robert = rockerbogie(dims, R, 1200);
Lr = [400, 600];
Ll = [180, 600];
lr = [155, 300];
ll = [155, 300];
db = [155, 200];
hb = [25, 200];
lims = [400, 600; 180, 600; 155, 300; 155, 300; 155, 200; 25, 200];
robert = robert.Optimize(1, a, R, lims, 3, 8);
%robert.CompareCG(a, 1000)
endmessage = ['Total run time: ', num2str(toc), ' seconds'];
disp(endmessage);
disp(robert.x(1,1,1));
disp(robert.x(1,2,1));