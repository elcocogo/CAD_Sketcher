import math

from bpy.types import Gizmo, GizmoGroup
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Vector
from mathutils.geometry import intersect_point_line

from .. import global_data
from ..declarations import GizmoGroups, Gizmos
from ..drawing import frame_cache, selection
from ..model.types import SlvsDistance
from ..utilities.view import get_scale_from_pos
from .base import ConstraintGenericGGT, ConstraintGizmoGeneric
from .utilities import (
    SELECT_TOLERANCE_PX,
    closest_distance_to_polyline,
    draw_arrow_shape,
    get_arrow_size,
    get_overshoot,
)


class VIEW3D_GGT_slvs_distance(GizmoGroup, ConstraintGenericGGT):
    bl_idname = GizmoGroups.Distance
    bl_label = "Distance Constraint Gizmo Group"

    type = SlvsDistance.type
    gizmo_type = Gizmos.Distance


class VIEW3D_GT_slvs_distance(Gizmo, ConstraintGizmoGeneric):
    bl_idname = Gizmos.Distance
    type = SlvsDistance.type

    # No draw_select: Blender only calls test_select when draw_select is
    # absent from the class entirely (see wm_gizmo_map.cc), and the analytic
    # test_select below is what right-click needs to find this constraint.

    bl_target_properties = (
        {
            "id": "offset",
            "type": "FLOAT",
            "array_length": 1,
        },
    )

    __slots__ = (
        "custom_shape",
        "index",
        "_shape_sig",
    )

    def test_select(self, context, location):
        """Analytic hit-test against the dimension's main line (not the
        decorative arrowheads/helplines), so right-click's generic keymap can
        find which constraint the cursor is over -- Blender's default GPU
        picking (draw_select) can't be observed from Python. Overriding this
        replaces that GPU picking for this gizmo entirely, left-click included.
        """
        if global_data.stateful_op_running:
            return -1
        constr = self._get_constraint(context)
        if not constr or not constr.visible:
            return -1

        ui_scale = context.preferences.system.ui_scale
        half_dist = constr.value / 2 / ui_scale
        offset = self.target_get_value("offset")
        p1 = self.matrix_world @ Vector((-half_dist, offset, 0.0))
        p2 = self.matrix_world @ Vector((half_dist, offset, 0.0))

        region, rv3d = context.region, context.region_data
        p1_2d = location_3d_to_region_2d(region, rv3d, p1)
        p2_2d = location_3d_to_region_2d(region, rv3d, p2)
        if p1_2d is None or p2_2d is None:
            return -1

        cursor = Vector(location)
        dist = closest_distance_to_polyline(cursor, (p1_2d, p2_2d))
        hit = dist < SELECT_TOLERANCE_PX

        # Same tracking as the value gizmo's test_select, for the same reason.
        key = (self.type, self.index)
        if hit:
            selection.constraint_hover = key
        elif selection.constraint_hover == key:
            selection.constraint_hover = None

        return 0 if hit else -1

    def _get_helplines(self, context, constr, scale_1, scale_2):
        ui_scale = context.preferences.system.ui_scale
        dist = constr.value / 2 / ui_scale
        offset = self.target_get_value("offset")
        entity1, entity2 = constr.ref(1), constr.ref(2)
        if entity1 is None or entity2 is None:
            return ((0, 0, 0),) * 4
        if entity1.is_line():
            entity1, entity2 = entity1.p1, entity1.p2

        # Get constraints points in local space and adjust helplines
        # based on their position
        sketch = frame_cache.active_sketch(context)
        basis = frame_cache.dimension_basis(sketch, constr) if sketch else None
        mat_inv = (basis if basis is not None else constr.matrix_basis()).inverted()

        def get_local(point):
            return (mat_inv @ point.to_3d()) / ui_scale

        # Store the two endpoints of the helplines in local space
        points_local = []

        # Add endpoint for entity1 helpline
        if entity1.is_curve():
            centerpoint = entity1.ct.co

            if entity2.is_point():
                targetpoint = entity2.co
            elif entity2.is_line():
                targetpoint, _ = intersect_point_line(
                    centerpoint, entity2.p1.co, entity2.p2.co
                )
            elif entity2.is_curve():
                targetpoint = entity2.ct.co
            else:
                # TODO: Handle the case for SlvsWorkplane
                targetpoint = centerpoint

            targetvec = targetpoint - centerpoint
            if targetvec.length:
                points_local.append(
                    get_local(
                        centerpoint + entity1.radius * targetvec / targetvec.length
                    )
                )
            else:
                points_local.append(get_local(centerpoint))

        else:
            points_local.append(get_local(entity1.location))

        # Add endpoint for entity2 helpline
        if entity2.is_curve():
            # Edge of the second curve facing the first (line of centres).
            centervec = entity2.ct.co - entity1.ct.co
            if centervec.length:
                edge = entity2.ct.co - entity2.radius * centervec / centervec.length
            else:
                edge = entity2.ct.co
            points_local.append(get_local(edge))

        elif entity2.is_point():
            points_local.append(get_local(entity2.location))

        elif entity2.is_line():
            line_points = (
                get_local(entity2.p1.location),
                get_local(entity2.p2.location),
            )
            line_points_side = [pos.y - offset > 0 for pos in line_points]

            x = math.copysign(dist, line_points[0].x)
            y = offset

            if line_points_side[0] != line_points_side[1]:
                # Distance line is between line points
                y = offset
            else:
                # Get the closest point
                points_delta = [abs(p.y - offset) for p in line_points]
                i = int(points_delta[0] > points_delta[1])
                y = line_points[i].y
            points_local.append(Vector((x, y, 0.0)))

        # Pick the points based on their x location
        if len(points_local) < 2:
            return ((0, 0, 0),) * 4
        if points_local[0].x > points_local[1].x:
            point_right, point_left = points_local
        else:
            point_right, point_left = reversed(points_local)

        overshoot_1 = offset + get_overshoot(scale_1, point_left.y - offset)
        overshoot_2 = offset + get_overshoot(scale_2, point_right.y - offset)

        return (
            (-dist, overshoot_1, 0.0),
            (-dist, point_left.y, 0.0),
            (dist, overshoot_2, 0.0),
            (dist, point_right.y, 0.0),
        )

    def _create_shape(self, context, constr, select=False):
        rv3d = context.region_data
        ui_scale = context.preferences.system.ui_scale

        half_dist = constr.value / 2 / ui_scale
        offset = self.target_get_value("offset")
        outset = constr.draw_outset

        p1 = Vector((-half_dist, offset, 0.0))
        p2 = Vector((half_dist, offset, 0.0))
        if not constr.text_inside(ui_scale):
            p1, p2 = p2, p1
        p1_global, p2_global = [self.matrix_world @ p for p in (p1, p2)]

        scale_1, scale_2 = [get_scale_from_pos(p, rv3d) for p in (p1_global, p2_global)]

        arrow_1 = get_arrow_size(half_dist, scale_1)
        arrow_2 = get_arrow_size(half_dist, scale_2)

        if constr.text_inside(ui_scale):
            coords = (
                *draw_arrow_shape(
                    p1, p1 + Vector((arrow_1[0], 0, 0)), arrow_1[1], is_3d=True
                ),
                p1,
                p2,
                *draw_arrow_shape(
                    p2, p2 - Vector((arrow_2[0], 0, 0)), arrow_2[1], is_3d=True
                ),
                *(
                    self._get_helplines(context, constr, scale_1, scale_2)
                    if not select
                    else ()
                ),
            )
        else:  # the same thing, but with a little jitter to the outside
            coords = (
                *draw_arrow_shape(
                    p1, p1 + Vector((arrow_1[0], 0, 0)), arrow_1[1], is_3d=True
                ),
                p1,
                # jitter back and forth to extend leader line for
                # text_outside case but it is unnecessary work for
                # text_inside case
                Vector((outset, offset, 0)),
                p1,
                p2,
                *draw_arrow_shape(
                    p2, p2 - Vector((arrow_2[0], 0, 0)), arrow_2[1], is_3d=True
                ),
                *(
                    self._get_helplines(context, constr, scale_1, scale_2)
                    if not select
                    else ()
                ),
            )

        self.custom_shape = self.new_custom_shape("LINES", coords)
