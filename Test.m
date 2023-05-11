clc; clear; close all;

%% Assign
eyeRobot = Robot(Const.LINKAGE_LENGTHS, Const.UPPER_LINKAGE_LOCATION, Const.LOWER_LINKAGE_LOCATION, Const.BALL_JOINT_OFFSET);
upper_linkage = five_bar_linkage(Const.LINKAGE_LENGTHS, Const.UPPER_LINKAGE_LOCATION);
lower_linkage = five_bar_linkage(Const.LINKAGE_LENGTHS, Const.LOWER_LINKAGE_LOCATION);
eyeRobot.upper_linkage = upper_linkage;
eyeRobot.lower_linkage = lower_linkage;

motor_upper_l = deg2rad(114.16); %(50, 50)
motor_upper_r = deg2rad(-19.86);
motor_lower_l = deg2rad(129.77); %(40, 40)
motor_lower_r = deg2rad(-30.25);

motors_state = [motor_upper_l motor_upper_r motor_lower_l motor_lower_r];
[upper_linkage_state, lower_linkage_state, ~, point_back_l, point_back_r, point_intersect] = eyeRobot.fk_linkages(motors_state);
five_bar_linkage_state = [upper_linkage_state; lower_linkage_state];
[cart_position, cart_configuration] = eyeRobot.fk_cart(five_bar_linkage_state);
zt = 5;
[tool_position] = eyeRobot.fk_tool(cart_position, cart_configuration, zt);

%% plot
upper_ground_l = Const.UPPER_LINKAGE_LOCATION;
upper_back_l = [point_back_l(1, 1), point_back_l(1, 2), Const.UPPER_LINKAGE_LOCATION(3)];
upper_intersect = [point_intersect(1, 1), point_intersect(1, 2), Const.UPPER_LINKAGE_LOCATION(3)];
upper_extended = [upper_linkage_state(1), upper_linkage_state(2), Const.UPPER_LINKAGE_LOCATION(3)];
upper_back_r = [point_back_r(1, 1), point_back_r(1, 2), Const.UPPER_LINKAGE_LOCATION(3)];
upper_ground_r = [Const.UPPER_LINKAGE_LOCATION(1)+Const.LINKAGE_LENGTHS(1), Const.UPPER_LINKAGE_LOCATION(2), Const.UPPER_LINKAGE_LOCATION(3)];

lower_ground_l = Const.LOWER_LINKAGE_LOCATION;
lower_back_l = [point_back_l(2, 1), point_back_l(2, 2), Const.LOWER_LINKAGE_LOCATION(3)];
lower_intersect = [point_intersect(2, 1), point_intersect(2, 2), Const.LOWER_LINKAGE_LOCATION(3)];
lower_extended = [lower_linkage_state(1), lower_linkage_state(2), Const.LOWER_LINKAGE_LOCATION(3)];
lower_back_r = [point_back_r(2, 1), point_back_r(2, 2), Const.LOWER_LINKAGE_LOCATION(3)];
lower_ground_r = [Const.LOWER_LINKAGE_LOCATION(1)+Const.LINKAGE_LENGTHS(1), Const.LOWER_LINKAGE_LOCATION(2), Const.LOWER_LINKAGE_LOCATION(3)];

upper_layer_points = [upper_ground_l; upper_back_l; upper_intersect; 
                      upper_extended; upper_intersect; upper_back_r; upper_ground_r];
lower_layer_points = [lower_ground_l; lower_back_l; lower_intersect; 
                      lower_extended; lower_intersect; lower_back_r; lower_ground_r];

cart_points = [upper_extended; lower_extended; cart_position; tool_position];

% cart, tool, cart_plane

figure
% upper layer
plot3(upper_layer_points(:, 1), upper_layer_points(:, 2), upper_layer_points(:, 3), 'linewidth', 2);
hold on
fill3(upper_layer_points(:, 1), upper_layer_points(:, 2), upper_layer_points(:, 3), [0 0.4470 0.7410], "facealpha", .5, "linestyle", "none");
hold on

% lower layer
plot3(lower_layer_points(:, 1), lower_layer_points(:, 2), lower_layer_points(:, 3), 'color', [0.8500 0.3250 0.0980], 'linewidth', 2);
hold on
fill3(lower_layer_points(:, 1), lower_layer_points(:, 2), lower_layer_points(:, 3), [0.8500 0.3250 0.0980], "facealpha", .5, "linestyle", "none");
hold on

% tool
plot3(cart_points(:, 1), cart_points(:, 2), cart_points(:, 3), 'k', 'linewidth', 2);


box on; grid on; grid minor;
