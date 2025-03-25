function wb_vacuum_gripper_enable_presence(tag, sampling_period)
% Usage: wb_vacuum_gripper_enable_presence(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/vacuum_gripper">here</a>

calllib('libController', 'wb_vacuum_gripper_enable_presence', tag, sampling_period);
