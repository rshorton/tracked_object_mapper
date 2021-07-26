# tracked_object_mapper

ROS 2 helper for the **tracker** node of the **robot_head** package.  This node transforms the position of the
tracked object published on /head/tracked to map coordinates and publishes to /tracked_object_map_position.

This helper is needed since it is not currently possible (as of 4/2021 Rolling) to perform this mapping
with a python node.
