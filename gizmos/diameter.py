from bpy.types import Gizmo, GizmoGroup
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Vector

from .. import global_data
from ..declarations import GizmoGroups, Gizmos
from ..drawing import selection
from ..model.types import SlvsDiameter
from ..utilities.constants import HALF_TURN
from ..utilities.math import pol2cart
from ..utilities.view import get_scale_from_pos
from .base import ConstraintGenericGGT, ConstraintGizmoGeneric
from .utilities import (
    SELECT_TOLERANCE_PX,
    closest_distance_to_polyline,
    draw_arrow_shape,
    get_arrow_size,
)


class VIEW3D_GGT_slvs_diameter(GizmoGroup, ConstraintGenericGGT):
    bl_idname = GizmoGroups.Diameter
    bl_label = "Diameter Gizmo Group"

    type = SlvsDiameter.type
    gizmo_type = Gizmos.Diameter


class VIEW3D_GT_slvs_diameter(Gizmo, ConstraintGizmoGeneric):
    bl_idname = Gizmos.Diameter
    type = SlvsDiameter.type

    # No draw_select -- see VIEW3D_GT_slvs_distance for why (analytic
    # test_select needed instead, for right-click support).

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
        """Analytic hit-test against the dimension's leader line(s) (not the
        decorative arrowheads) -- see VIEW3D_GT_slvs_distance.test_select for
        why this replaces Blender's default GPU-based picking entirely.
        """
        if global_data.stateful_op_running:
            return -1
        constr = self._get_constraint(context)
        if not constr or not constr.visible:
            return -1

        ui_scale = context.preferences.system.ui_scale
        angle = constr.leader_angle
        offset = constr.draw_offset / ui_scale
        dist = constr.radius / ui_scale
        rv3d = context.region_data

        p1 = pol2cart(-dist, angle)
        p2 = pol2cart(dist, angle)

        if constr.setting:
            # RADIUS_MODE: a single segment, inside (to the center) or
            # outside (to the leader/text point).
            if constr.text_inside():
                points_local = (p2, Vector((0.0, 0.0)))
            else:
                points_local = (p2, pol2cart(offset, angle))
        else:
            # DIAMETER_MODE: a straight segment inside, or a bent leader
            # path (matching _create_shape) outside.
            if constr.text_inside():
                points_local = (p1, p2)
            else:
                p2_global = self.matrix_world @ p2.to_3d()
                arrow_2 = get_arrow_size(dist, get_scale_from_pos(p2_global, rv3d))
                points_local = (
                    p2,
                    pol2cart(offset, angle),
                    pol2cart(dist + (3 * arrow_2[0]), angle + HALF_TURN),
                    p1,
                )

        region = context.region
        points_2d = []
        for p in points_local:
            screen = location_3d_to_region_2d(
                region, rv3d, self.matrix_world @ p.to_3d()
            )
            if screen is not None:
                points_2d.append(screen)
        if len(points_2d) < 2:
            return -1

        cursor = Vector(location)
        hit = closest_distance_to_polyline(cursor, points_2d) < SELECT_TOLERANCE_PX

        # Same tracking as the value gizmo's test_select, for the same reason.
        key = (self.type, self.index)
        if hit:
            selection.constraint_hover = key
        elif selection.constraint_hover == key:
            selection.constraint_hover = None

        return 0 if hit else -1

    def _create_shape(self, context, constr, select=False):
        ui_scale = context.preferences.system.ui_scale
        angle = constr.leader_angle
        offset = constr.draw_offset / ui_scale
        dist = constr.radius / ui_scale

        rv3d = context.region_data

        p1 = pol2cart(-dist, angle)
        p2 = pol2cart(dist, angle)

        p1_global, p2_global = [self.matrix_world @ p.to_3d() for p in (p1, p2)]
        scale_1, scale_2 = [get_scale_from_pos(p, rv3d) for p in (p1_global, p2_global)]

        arrow_1 = get_arrow_size(dist, scale_1)
        arrow_2 = get_arrow_size(dist, scale_2)

        if constr.setting:
            # RADIUS_MODE:
            #   drawn inside and outside as a single segment
            if constr.text_inside():
                coords = (
                    *draw_arrow_shape(
                        p2, pol2cart(dist - arrow_2[0], angle), arrow_2[1]
                    ),
                    p2,
                    (0, 0),
                )
            else:
                coords = (
                    *draw_arrow_shape(
                        p2, pol2cart(arrow_2[0] + dist, angle), arrow_2[1]
                    ),
                    p2,
                    pol2cart(offset, angle),
                )

        else:
            # DIAMETER_MODE:
            #   drawn inside as a single segment
            #   drawn outside as a 2-segment gizmo
            if constr.text_inside():
                coords = (
                    *draw_arrow_shape(
                        p1, pol2cart(arrow_2[0] - dist, angle), arrow_2[1]
                    ),
                    p1,
                    p2,
                    *draw_arrow_shape(
                        p2, pol2cart(dist - arrow_2[0], angle), arrow_2[1]
                    ),
                )
            else:
                coords = (
                    *draw_arrow_shape(
                        p2, pol2cart(arrow_1[0] + dist, angle), arrow_1[1]
                    ),
                    p2,
                    pol2cart(offset, angle),
                    pol2cart(
                        dist + (3 * arrow_2[0]), angle + HALF_TURN
                    ),  # limit length to 3 arrowheads
                    p1,
                    *draw_arrow_shape(
                        p1,
                        pol2cart(dist + arrow_2[0], angle + HALF_TURN),
                        arrow_2[1],
                    ),
                )

        self.custom_shape = self.new_custom_shape("LINES", coords)
