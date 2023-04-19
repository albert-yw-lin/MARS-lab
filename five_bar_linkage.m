classdef five_bar_linkage
   properties
       % linkage length and height in mm (constant)
       GROUND
       BACK_L
       BACK_R
       FRONT_L
       FRONT_R
       FRONT_L_EXTENDED % extended length of the front left arm
       X_OFFSET
       Y_OFFSET
   end
   
   methods
       function obj = five_bar_linkage(LINKAGE_LENGTHS, LINKAGE_LOCATION)
           % LINKAGE_LENGTHS: 6 elements array which contains linkage length argument
           obj.GROUND = LINKAGE_LENGTHS(1);
           obj.BACK_L = LINKAGE_LENGTHS(2);
           obj.BACK_R = LINKAGE_LENGTHS(3);
           obj.FRONT_L = LINKAGE_LENGTHS(4);
           obj.FRONT_R = LINKAGE_LENGTHS(5);
           obj.FRONT_L_EXTENDED = LINKAGE_LENGTHS(6);
           obj.X_OFFSET = LINKAGE_LOCATION(1);
           obj.Y_OFFSET = LINKAGE_LOCATION(2);
       end

       function [x_extended, y_extended, rad_front_l] = fk(rad_back_l, rad_back_r)
           % based on "5-Bar Linkage Kinematic Solver and Simulator"

           % coordinates of back arms
           x_back_l = obj.BACK_L*cos(rad_back_l); % (=xr1)
           y_back_l = obj.BACK_L*sin(rad_back_l); % (=yr1)
           x_back_r = obj.BACK_R*cos(rad_back_r) + obj.GROUND; % (=xr2)
           y_back_r = obj.BACK_R*sin(rad_back_r); % (=yr2)

           % some calculation process based on the paper
           v1 = (y_back_l - y_back_r)/(x_back_r - x_back_l);
           v2 = (obj.FRONT_L^2 - obj.FRONT_R^2 - obj.BACK_L^2 + x_back_r^2 + y_back_r^2)/(2*(x_back_r-x_back_l));
           v3 = 1 + v1^2;
           v4 = 2*(v1*v2 - v1*x_back_l - y_back_l);
           v5 = obj.BACK_L^2 - obj.FRONT_L^2 - 2*v2*x_back_l + v2^2;
           
           % position of the intersection of 5-bar linkage
           y_intersect = (-v4 + sqrt(v4^2 - 4*v3*v5))/2*v3; % positive solution
           % y_intersect = (-v4 - sqrt(v4^2 - 4*v3*v5))/2*v3; % negative solution
           x_intersect = v1*y_intersect + v2;

           % rad_front_l: orientation of the front left arm
           unit_vector_front_l = [x_intersect - x_back_l, y_intersect - y_back_l] ./ norm([x_intersect - x_back_l, y_intersect - y_back_l]);
           rad_front_l = atan2(unit_vector_front_l(1), unit_vector_front_l(0));

           % position of the extended front left arm
           x_extended = x_intersect + obj.FRONT_L_EXTENDED*unit_vector_front_l(0);
           y_extended = y_intersect + obj.FRONT_L_EXTENDED*unit_vector_front_l(1);

           % add the XY offset when in the robot system
           x_extended = x_extended + obj.X_OFFSET;
           y_extended = y_extended + obj.Y_OFFSET;
       end
   end
end