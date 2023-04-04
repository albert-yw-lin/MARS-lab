classdef Const
    properties (Constant)

        % [GROUND, BACK_L, BACK_R, FRONT_L, FRONT_R, FRONT_L_EXTENDED]
        LINKAGE_LENGTHS = [41 45 45 69 69 15]

        % [X, Y, Z]
        UPPER_LINKAGE_LOCATION = [0 -10 29]
        LOWER_LINKAGE_LOCATION = [0 0 0]

        % ball joint offset
        RAD_BALL_JOINT_OFFSET = deg2rad(134.08)
        LENGTH_BALL_JOINT_OFFSET = 28
        BALL_JOINT_OFFSET = [Const.RAD_BALL_JOINT_OFFSET Const.LENGTH_BALL_JOINT_OFFSET]
    end
end
