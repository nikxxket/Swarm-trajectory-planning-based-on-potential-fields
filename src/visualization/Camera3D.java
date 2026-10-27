package visualization;

import java.awt.geom.Point2D;

import logic.Vec3;

public class Camera3D {
    public double azimuth = Math.toRadians(45);
    public double elevation = Math.toRadians(35);
    public double distance = 45;
    public Vec3 center;

    public Vec3 position;
    Vec3 forward, right, up;

    public Camera3D(Vec3 center) { this.center = center; update(); }

    public void update() {
        double ce = Math.cos(elevation), se = Math.sin(elevation);
        double ca = Math.cos(azimuth), sa = Math.sin(azimuth);

        position = new Vec3(
                center.x + distance * ce * ca,
                center.y + distance * ce * sa,
                center.z + distance * se);

        forward = center.sub(position).norm();
        Vec3 worldUp = new Vec3(0, 0, 1);
        right = forward.cross(worldUp).norm();
        if (right.len() < 1e-6) right = new Vec3(1, 0, 0);
        up = right.cross(forward).norm();
    }

    public Point2D.Double project(Vec3 world, int screenW, int screenH) {
        Vec3 v = world.sub(position);
        double depth = v.dot(forward);
        if (depth < 0.5) return null;
        double x = v.dot(right);
        double y = v.dot(up);
        double focal = 600.0;
        return new Point2D.Double(screenW / 2.0 + focal * x / depth,
                screenH / 2.0 - focal * y / depth);
    }

    public double depthOf(Vec3 world) {
        return world.sub(position).dot(forward);
    }
}