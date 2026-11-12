"""ROS-independent validation; map geometry must still be checked by an operator."""
import math
import uuid


def finite_number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be a number')
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(name + ' must be finite' + (' and positive' if positive else ''))
    return value


def validate_config(config, live=False):
    if not isinstance(config, dict):
        raise ValueError('Config must be an object')
    map_id = config.get('approved_map_id')
    if map_id != 'REPLACE_AFTER_MAP_READY':
        if not isinstance(map_id, str):
            raise ValueError('approved_map_id must be a map UUID or REPLACE_AFTER_MAP_READY')
        try:
            uuid.UUID(map_id)
        except ValueError as exc:
            raise ValueError('approved_map_id must be a map UUID') from exc
    elif live:
        raise ValueError('Set approved_map_id before live execution')
    finite_number(config.get('navigation_timeout_seconds', 120), 'navigation timeout', True)
    waypoints = config.get('waypoints')
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError('waypoints must be a nonempty list')
    for index, wp in enumerate(waypoints, 1):
        if not isinstance(wp, dict):
            raise ValueError('Waypoint %d must be an object' % index)
        for key in ('x', 'y', 'orientation_deg'):
            finite_number(wp.get(key), 'Waypoint %d %s' % (index, key))
        if not -180 <= wp['orientation_deg'] <= 180:
            raise ValueError('Waypoint %d orientation must be in [-180, 180] degrees' % index)
        if type(wp.get('oven', False)) is not bool:
            raise ValueError('Waypoint %d oven must be a boolean' % index)
    return config
