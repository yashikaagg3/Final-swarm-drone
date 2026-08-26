"""
Divides the rectangular mapping area into approximately equal regions.

Arranged as a grid as close to square as possible (4 -> 2x2, 6 -> 2x3,
8 -> 2x4, 9 -> 3x3, ...). Pure geometry, no ROS/Gazebo dependency, so it
can be unit tested directly.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Region:
    drone_id: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def area(self):
        return (self.max_x - self.min_x) * (self.max_y - self.min_y)

    def center(self):
        return ((self.min_x + self.max_x) / 2.0, (self.min_y + self.max_y) / 2.0)

    def contains(self, x, y, margin=0.0):
        return (self.min_x + margin <= x <= self.max_x - margin
                and self.min_y + margin <= y <= self.max_y - margin)


def _grid_shape(n):
    """Pick (rows, cols) with rows*cols >= n, as close to a square as possible."""
    rows = int(math.floor(math.sqrt(n)))
    rows = max(rows, 1)
    cols = int(math.ceil(n / rows))
    return rows, cols


def allocate_regions(num_drones, min_x, max_x, min_y, max_y):
    """
    Return `num_drones` Region objects covering [min_x,max_x]x[min_y,max_y].

    Regions never overlap and (for grid cell counts that divide evenly
    into num_drones) have equal area. Drone ids are assigned row-major,
    0..num_drones-1.
    """
    if num_drones < 1:
        raise ValueError(f'num_drones must be >= 1, got {num_drones}')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError(
            f'invalid mapping area: x=[{min_x},{max_x}] y=[{min_y},{max_y}]')

    rows, cols = _grid_shape(num_drones)
    total_width = max_x - min_x
    total_height = max_y - min_y

    # Cells are filled row-major; a grid can have more cells than drones
    # (e.g. 5 drones -> 2x3 = 6 cells), so the last row may be short one cell -
    # its cells are simply widened to still cover the full width, keeping
    # every assigned region's area close to total_area / num_drones.
    regions = []
    drone_id = 0
    for row in range(rows):
        cells_remaining = num_drones - drone_id
        rows_remaining = rows - row
        cols_this_row = min(cols, cells_remaining) if rows_remaining == 1 else cols
        cols_this_row = max(cols_this_row, 1)

        row_min_y = min_y + row * (total_height / rows)
        row_max_y = min_y + (row + 1) * (total_height / rows)

        for col in range(cols_this_row):
            if drone_id >= num_drones:
                break
            cell_min_x = min_x + col * (total_width / cols_this_row)
            cell_max_x = min_x + (col + 1) * (total_width / cols_this_row)
            regions.append(Region(
                drone_id=drone_id,
                min_x=cell_min_x, max_x=cell_max_x,
                min_y=row_min_y, max_y=row_max_y,
            ))
            drone_id += 1

    return regions
