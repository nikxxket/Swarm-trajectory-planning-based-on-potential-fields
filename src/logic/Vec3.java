package logic;

public class Vec3 {
    public double x, y, z;
    public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    public Vec3 add(Vec3 o) { return new Vec3(x + o.x, y + o.y, z + o.z); }
    public Vec3 sub(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }
    public Vec3 mul(double k) { return new Vec3(x * k, y * k, z * k); }
    public double dot(Vec3 o) { return x * o.x + y * o.y + z * o.z; }
    public Vec3 cross(Vec3 o) {
        return new Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    public double len() { return Math.sqrt(x * x + y * y + z * z); }
    public Vec3 norm() { double l = len(); return l < 1e-9 ? new Vec3(0, 0, 0) : mul(1 / l); }
}