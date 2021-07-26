Helper for tracker node of robot_head package that transforms the position of the
tracked object published on /head/tracked to map coordinates and publishes to
/tracked_object_map_position.

This helper is needed since it is not currently possible to perform this mapping
with a python node.
