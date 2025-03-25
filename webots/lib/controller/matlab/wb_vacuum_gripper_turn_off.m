function wb_vacuum_gripper_turn_off(tag)
% Usage: wb_vacuum_gripper_turn_off(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/vacuum_gripper">here</a>

calllib('libController', 'wb_vacuum_gripper_turn_off', tag);
