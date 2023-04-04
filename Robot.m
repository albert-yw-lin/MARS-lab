classdef Robot
   properties
       % two five_bar_linkage instances
       upper_linkage
       lower_linkage
       
       UPPER_HEIGHT 
       LOWER_HEIGHT
       RAD_BALL_JOINT % fixed rad angle between lower extended linkage and decomposed ball joint
       LENGTH_BALL_JOINT
   end
   
   methods
       function obj = Robot(LINKAGE_LENGTHS, UPPER_LINKAGE_LOCATION, LOWER_LINKAGE_LOCATION, BALL_JOINT_OFFSET)
           % *_LINKAGE_LENGTHS: [GROUND, BACK_L, BACK_R, FRONT_L, FRONT_R, FRONT_L_EXTENDED]
           % *_LINKAGE_LOCATION: [X, Y, Z]. location of left motor of the linkage with respect to the origin of the robot system
           obj.upper_linkage = five_bar_linkage(LINKAGE_LENGTHS, UPPER_LINKAGE_LOCATION(1:2));
           obj.lower_linkage = five_bar_linkage(LINKAGE_LENGTHS, LOWER_LINKAGE_LOCATION(1:2));

           obj.UPPER_HEIGHT = UPPER_LINKAGE_LOCATION(3);
           obj.LOWER_HEIGHT = LOWER_LINKAGE_LOCATION(3);
           obj.RAD_BALL_JOINT = BALL_JOINT_OFFSET(1);
           obj.LENGTH_BALL_JOINT = BALL_JOINT_OFFSET(2);
       end

       function [upper_linkage_state, lower_linkage_state, rad_ball_joint] = fk_linkages(motors_state)
           % motor_*: [motor_upper_l, motor_upper_r, motor_lower_l, motor_lower_r] (rad)
           % *_linkage_state: [x, y, angle(rad)] of extended arm
           [x_extended_upper, y_extended_upper, ~] = obj.upper_linkage.fk(motors_state(1), motors_state(2));
           [x_extended_lower, y_extended_lower, rad_front_l_lower] = obj.lower_linkage.fk(motors_state(3), motors_state(4));

           % ball_joint_angle & lower linkage state
           rad_ball_joint = rad_front_l_lower + (pi - obj.RAD_BALL_JOINT);
           x_lower = x_extended_lower + obj.LENGTH_BALL_JOINT * cos(rad_ball_joint);
           y_lower = y_extended_lower + obj.LENGTH_BALL_JOINT * sin(rad_ball_joint);
           lower_linkage_state = [x_lower, y_lower];

           % upper linkage state
           x_upper = x_extended_upper + obj.LENGTH_BALL_JOINT * cos(rad_ball_joint);
           y_upper = y_extended_upper + obj.LENGTH_BALL_JOINT * sin(rad_ball_joint);
           upper_linkage_state = [x_upper, y_upper];
       end

       function [cart_state] = fk_cart(motors_state)
           % motor_*: in radian
           %%% TODO %%%
       end
   end
end