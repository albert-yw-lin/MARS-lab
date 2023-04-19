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
       
       function [J, J_cart] = fk_jacobian(motors_state)
           % based on "5-Bar Linkage Kinematic Solver and Simulator"
           syms motor_upper_l motor_upper_r motor_lower_l motor_lower_r
           
           %%%%%%%%%%%%% Jacobian of upper and lower linkage %%%%%%%%%%%%%
           % coordinates of back arms
           x_back_l = obj.lower_linkage.BACK_L*cos(motor_lower_l); % (=xr1)
           y_back_l = obj.lower_linkage.BACK_L*sin(motor_lower_l); % (=yr1)
           x_back_r = obj.lower_linkage.BACK_R*cos(motor_lower_r) + obj.upper_linkage.GROUND; % (=xr2)
           y_back_r = obj.lower_linkage.BACK_R*sin(motor_lower_r); % (=yr2)

           % some calculation process based on the paper
           v1 = (y_back_l - y_back_r)/(x_back_r - x_back_l);
           v2 = (obj.lower_linkage.FRONT_L^2 - obj.lower_linkage.FRONT_R^2 - obj.lower_linkage.BACK_L^2 + x_back_r^2 + y_back_r^2)/(2*(x_back_r-x_back_l));
           v3 = 1 + v1^2;
           v4 = 2*(v1*v2 - v1*x_back_l - y_back_l);
           v5 = obj.lower_linkage.BACK_L^2 - obj.lower_linkage.FRONT_L^2 - 2*v2*x_back_l + v2^2;

           % position of the intersection of 5-bar linkage
           y_intersect = (-v4 + sqrt(v4^2 - 4*v3*v5))/2*v3; % positive solution
           % y_intersect = (-v4 - sqrt(v4^2 - 4*v3*v5))/2*v3; % negative solution
           x_intersect = v1*y_intersect + v2;

           % rad_front_l: orientation of the front left arm
           unit_vector_front_l = [x_intersect - x_back_l, y_intersect - y_back_l] ./ norm([x_intersect - x_back_l, y_intersect - y_back_l]);
           rad_front_l = atan2(unit_vector_front_l(1), unit_vector_front_l(0));

           % position of the extended front left arm
           x_extended = x_intersect + obj.lower_linkage.FRONT_L_EXTENDED*unit_vector_front_l(0);
           y_extended = y_intersect + obj.lower_linkage.FRONT_L_EXTENDED*unit_vector_front_l(1);

           % add the XY offset when in the robot system (lower layer)
           x_extended_lower = x_extended + obj.lower_linkage.X_OFFSET;
           y_extended_lower = y_extended + obj.lower_linkage.Y_OFFSET;

           % add the XY offset when in the robot system (upper layer)
           x_extended_upper = subs(x_extended_lower, [motor_lower_l, motor_lower_r], [motor_upper_l, motor_upper_r]);
           y_extended_upper = subs(y_extended_lower, [motor_lower_l, motor_lower_r], [motor_upper_l, motor_upper_r]);
           x_extended_upper = x_extended_upper + obj.upper_linkage.X_OFFSET;
           y_extended_upper = y_extended_upper + obj.upper_linkage.Y_OFFSET;

           % ball_joint_angle & lower linkage state
           rad_ball_joint = rad_front_l + (pi - obj.lower_linkage.RAD_BALL_JOINT);
           x_lower = x_extended_lower + obj.lower_linkage.LENGTH_BALL_JOINT * cos(rad_ball_joint);
           y_lower = y_extended_lower + obj.lower_linkage.LENGTH_BALL_JOINT * sin(rad_ball_joint);
           lower_linkage_state = [x_lower, y_lower];

           % upper linkage state
           x_upper = x_extended_upper + obj.upper_linkage.LENGTH_BALL_JOINT * cos(rad_ball_joint);
           y_upper = y_extended_upper + obj.upper_linkage.LENGTH_BALL_JOINT * sin(rad_ball_joint);
           upper_linkage_state = [x_upper, y_upper];
           
           %%%%%%%%%%%%% Jacobian of the cart %%%%%%%%%%%%%
           upper_point = [upper_linkage_state(1); upper_linkage_state(2); UPPER_LINKAGE_LOCATION(3); 1];
           lower_point = [lower_linkage_state(1); lower_linkage_state(2); LOWER_LINKAGE_LOCATION(3); 1];

           % determine theta_yz and theta_xz
           vector_cart_direction = upper_point - lower_point;
           theta_yz = atan2(vector_cart_direction(2), vector_cart_direction(3)); % in radian
           theta_xz = atan2(vector_cart_direction(1), vector_cart_direction(3)); % in radian

           % determine the location of cart
           DH_universal_to_ball_joint = [-0.5*pi theta_xz lower_point(1) lower_point(2)];
           DH_ball_joint_to_cart = [-0.5*pi theta_yz 0 LENGTH_BALL_JOINT_TO_CART];

           trans_matrix_universal_to_ball_joint = DH2trans(DH_universal_to_ball_joint);
           trans_matrix_ball_joint_to_cart = DH2trans(DH_ball_joint_to_cart);
           trans_cart_offset = DH2trans(0, -0.5*pi, LENGTH_CART_OFFSET, 0);

           cart_position = trans_cart_offset * trans_matrix_ball_joint_to_cart * trans_matrix_universal_to_ball_joint * lower_point;           
           cart_position = cart_position.'; cart_position = cart_position(1, 1:3);
           
           %%%%%%%%%%%%% Output %%%%%%%%%%%%%
           % the analytical solution of jacobian
           J_analytical_upper = jacobian(upper_linkage_state, [motor_upper_l; motor_upper_r;motor_lower_l; motor_lower_r]);
           J_analytical_lower = jacobian(lower_linkage_state, [motor_lower_l; motor_lower_r]);
           J_cart = jacobian(cart_position, [motor_upper_l; motor_upper_r;motor_lower_l; motor_lower_r]);
           
           % if there is no input, then the output will be analytical solution
           % if the input is motors state, the output will be numerical solution
           if (nargin == 0)
               J = [J_analytical_upper J_analytical_lower];
           else
               % the numerical solution of jacobian
               J_numerical_upper = subs(J_analytical_upper, [motor_lower_l motor_lower_r motor_upper_l motor_upper_r], motors_state(1) motors_state(2) motors_state(3) motors_state(4)]);
               J_numerical_lower = subs(J_analytical_lower, [motor_lower_l motor_lower_r], [motors_state(3) motors_state(4)]);
               J = [J_numerical_upper J_numerical_lower];
               J_cart = subs(J_cart, [motor_lower_l motor_lower_r motor_upper_l motor_upper_r], motors_state(1) motors_state(2) motors_state(3) motors_state(4)]);
           end  
       end
       
       function transformation_matrix = DH2trans(DH_parameters)
            % DH_parameters: [alpha, theta, a, d] (mm)
            alpha = DH_parameters(1); theta = DH_parameters(2); a = DH_parameters(3); d = DH_parameters(4);
            transformation_matrix = [cos(theta) -sin(theta) 0 alpha;
                                     sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
                                     sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
                                     0 0 0 1];
       end
        
        function [cart_state] = fk_cart(five_bar_linkage_state)
            % five_bar_linkage_state: [upper_linkage_state, lower_linkage_state]
            % cart_state: [cart_position, theta_xz, theta_yz]
            % cart_position: [x_cart, y_cart, z_cart]
            upper_linkage_state = five_bar_linkage_state(1, :);
            lower_linkage_state = five_bar_linkage_state(2, :);
            upper_point = [upper_linkage_state(1), upper_linkage_state(2), UPPER_LINKAGE_LOCATION(3), 1];
            lower_point = [lower_linkage_state(1), lower_linkage_state(2), LOWER_LINKAGE_LOCATION(3), 1];

            % determine theta_yz and theta_xz
            vector_cart_direction = upper_point - lower_point;
            theta_yz = atan2(vector_cart_direction(2), vector_cart_direction(3)); % in radian
            theta_xz = atan2(vector_cart_direction(1), vector_cart_direction(3)); % in radian

            % determine the location of cart
            DH_universal_to_ball_joint = [-0.5*pi theta_xz lower_point(1) lower_point(2)];
            DH_ball_joint_to_cart = [-0.5*pi theta_yz 0 LENGTH_BALL_JOINT_TO_CART];

            trans_matrix_universal_to_ball_joint = DH2trans(DH_universal_to_ball_joint);
            trans_matrix_ball_joint_to_cart = DH2trans(DH_ball_joint_to_cart);
            trans_cart_offset = DH2trans(0, -0.5*pi, LENGTH_CART_OFFSET, 0);

            cart_position = trans_cart_offset * trans_matrix_ball_joint_to_cart * trans_matrix_universal_to_ball_joint * lower_point;
            cart_state = [cart_position, theta_xz, theta_yz];
        end
   end
end