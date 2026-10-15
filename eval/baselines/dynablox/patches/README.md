# Patches to Dynablox (output only)

`export_cluster_id.patch`: in `MotionVisualizer::visualizeClusters` the box marker's `id` is set to the tracked
`cluster.id` instead of a running index. Dynablox already tracks clusters across frames (`tracking.max_tracking_distance`);
upstream only exposes that id as a colour (20-colour wheel), so it could not be read back. Detection, clustering and
tracking are untouched. Without the patch Dynablox would be scored on detection metrics only, like LV-DOT.

Build-only (in the Dockerfile): the `<depend>` lines for `ouster_ros` and `lidar_undistortion` are removed from
`dynablox_ros/package.xml`. They are used only by the upstream DOALS demo launch files (Ouster driver, motion
undistortion); no Dynablox source file includes them.
