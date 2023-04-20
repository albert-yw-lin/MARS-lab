clc; clear; close all;

upper_linkage = five_bar_linkage(Const.LINKAGE_LENGTHS, UPPER_LINKAGE_LOCATION);
lower_linkage = five_bar_linkage(Const.LINKAGE_LENGTHS, LOWER_LINKAGE_LOCATION);
eyeRobot = Robot(LINKAGE_LENGTHS, UPPER_LINKAGE_LOCATION, LOWER_LINKAGE_LOCATION, BALL_JOINT_OFFSET);
eyeRobot.upper_linkage = upper_linkage;
eyeRobot.upper_linkage = lower_linkage;
