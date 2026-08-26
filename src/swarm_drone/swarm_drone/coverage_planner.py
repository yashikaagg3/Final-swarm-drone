"""
Deterministic boustrophedon (lawnmower) coverage path generator.

Operates on a single Region. Pure geometry, no ROS/Gazebo dependency.
"""


def generate_coverage_path(region, waypoint_spacing, boundary_margin, altitude=0.0):
    """
    Return ordered (x, y, z) waypoints that sweep `region` back and forth.

    Stays `boundary_margin` inside the region's edges, with
    `waypoint_spacing` between sweep rows.
    """
    if waypoint_spacing <= 0:
        raise ValueError(f'waypoint_spacing must be > 0, got {waypoint_spacing}')
    if boundary_margin < 0:
        raise ValueError(f'boundary_margin must be >= 0, got {boundary_margin}')

    min_x = region.min_x + boundary_margin
    max_x = region.max_x - boundary_margin
    min_y = region.min_y + boundary_margin
    max_y = region.max_y - boundary_margin
    if max_x <= min_x or max_y <= min_y:
        raise ValueError(
            f'boundary_margin={boundary_margin} leaves no usable area in '
            f'region drone_id={region.drone_id}')

    epsilon = 1e-9
    rows = [min_y]
    y = min_y + waypoint_spacing
    while y < max_y - epsilon:
        rows.append(y)
        y += waypoint_spacing
    if rows[-1] < max_y - epsilon:
        rows.append(max_y)

    waypoints = []
    for i, row_y in enumerate(rows):
        if i % 2 == 0:
            waypoints.append((min_x, row_y, altitude))
            waypoints.append((max_x, row_y, altitude))
        else:
            waypoints.append((max_x, row_y, altitude))
            waypoints.append((min_x, row_y, altitude))
    return waypoints
