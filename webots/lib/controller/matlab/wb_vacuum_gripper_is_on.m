function result = wb_vacuum_gripper_is_on(tag)
% Usage: wb_vacuum_gripper_is_on(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/vacuum_gripper">here</a>

result = calllib('libController', 'wb_vacuum_gripper_is_on', tag);
