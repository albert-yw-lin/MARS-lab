classdef Const
    properties (Constant)

        % [GROUND, BACK_L, BACK_R, FRONT_L, FRONT_R, FRONT_L_EXTENDED]
        LINKAGE_LENGTHS = [41 45 45 69 69 15]

        % [X, Y, Z]
        UPPER_LINKAGE_LOCATION = [0 0 29]% 0 -10 29%%%%%%%%%%%%%%%%%%%%%%%%
        LOWER_LINKAGE_LOCATION = [0 0 0]

        % ball joint offset
        RAD_BALL_JOINT_OFFSET = deg2rad(134.08)
        LENGTH_BALL_JOINT_OFFSET = 12
        BALL_JOINT_OFFSET = [Const.RAD_BALL_JOINT_OFFSET Const.LENGTH_BALL_JOINT_OFFSET]
        
        % distance between ball joint and R-joint
        LENGTH_BALL_JOINT_TO_R_JOINT = 16;
        
       % distance between R-joint and the cart
        LENGTH_R_JOINT_TO_CART = 25.37;
    end
end
