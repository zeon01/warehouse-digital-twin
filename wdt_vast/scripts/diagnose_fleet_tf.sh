#!/usr/bin/env bash
# Fleet TF diagnostic — run on a vast.ai instance while a fleet sim is
# active. Captures the 8 observations the M5 expert (see
# `docs/m5-expert-consultation.md`) called out as the minimum to determine
# whether `/amr_0/tf` is reliably populated, whether frame IDs are
# prefixed, and whether pp_driver's TF listener actually subscribes to
# the namespaced topics.
#
# Usage (inside the instance, after the sim has booted):
#   source /opt/ros/humble/setup.bash
#   source /work/ros2_ws/install/setup.bash
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   bash /work/wdt_vast/scripts/diagnose_fleet_tf.sh > /tmp/fleet_tf_diag.log 2>&1

set +u

NS="${1:-amr_0}"
echo "==> Diagnosing namespace /${NS}/ at $(date -u +%FT%TZ)"
echo

echo "==> 1. Topics matching ${NS}|tf|odom"
ros2 topic list 2>&1 | grep -E "${NS}/(tf|tf_static|odom)" || echo "(no matching topics)"
echo

echo "==> 2. /${NS}/tf_static single message (map→odom expected)"
timeout 4 ros2 topic echo "/${NS}/tf_static" --once 2>&1 | head -25 || echo "(timeout / no message)"
echo

echo "==> 3. /${NS}/tf single message (odom→base_link expected); THE critical one"
timeout 5 ros2 topic echo "/${NS}/tf" --once 2>&1 | head -25 || echo "EMPTY — OG not ticking or targetPrims invalid"
echo

echo "==> 4. child_frame_id only — expecting raw 'base_link', NOT 'amr_0/base_link'"
timeout 4 ros2 topic echo "/${NS}/tf" --once --field "transforms[0].child_frame_id" 2>&1 | head -5 || echo "(no message)"
echo

echo "==> 5. pp_driver subscribers (should include /${NS}/tf and /${NS}/tf_static, NOT /tf, /tf_static)"
timeout 5 ros2 node info "/${NS}/pure_pursuit_driver" 2>&1 | sed -n '/Subscribers:/,/Service Servers:/p' || echo "(node not found)"
echo

echo "==> 6. tf2_echo map → base_link on /${NS}/tf"
timeout 5 ros2 run tf2_ros tf2_echo map base_link --topic "/${NS}/tf" 2>&1 | head -15 || echo "(timeout)"
echo

echo "==> 7. /${NS}/odom message rate (sanity that OG is producing anything)"
timeout 5 ros2 topic hz "/${NS}/odom" 2>&1 | head -5 || echo "(timeout or 0 Hz)"
echo

echo "==> 8. OG compute count must be inspected from kit (see expert §Q3)"
echo "    From inside Isaac Sim kit Python:"
echo "      import omni.graph.core as og"
echo "      node = og.Controller.node('/World/${NS}/.../ROS2PublishTransformTree')"
echo "      print(node.get_compute_count())"
echo "    If 0 after several seconds of play, the OG isn't ticking → World A."
echo
echo "==> diagnostic done at $(date -u +%FT%TZ)"
