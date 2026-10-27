package logic;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

public class Drone3D {
    public int x, y, z;
    public int[] goal;
    public int id;
    public List<Vec3> path = new ArrayList<>();
    public Color color;
    public double speed;

    public double progress = 0;
    public double totalLength = 0;
    public double[] cumLen = new double[0];

    public Drone3D(int x, int y, int z, int[] goal, int id, Color c, double speed) {
        this.x = x; this.y = y; this.z = z;
        this.goal = goal; this.id = id;
        this.color = c; this.speed = speed;
    }

    public void initPath() {
        if (path == null || path.size() < 2) {
            cumLen = new double[]{0};
            totalLength = 0; progress = 0; return;
        }
        cumLen = new double[path.size()];
        cumLen[0] = 0;
        for (int i = 1; i < path.size(); i++) {
            Vec3 a = path.get(i - 1), b = path.get(i);
            cumLen[i] = cumLen[i - 1] + a.sub(b).len();
        }
        totalLength = cumLen[cumLen.length - 1];
        progress = 0;
    }

    public boolean finished() { return path == null || totalLength <= 0 || progress >= totalLength; }

    public boolean advance() {
        if (finished()) return false;
        progress = Math.min(totalLength, progress + speed);
        return !finished();
    }

    public Vec3 getPosition() {
        if (path == null || path.isEmpty()) return new Vec3(x + 0.5, y + 0.5, z + 0.5);
        if (path.size() == 1 || totalLength == 0) return path.get(0);
        double s = progress;
        for (int i = 1; i < path.size(); i++) {
            if (cumLen[i] >= s) {
                double segLen = cumLen[i] - cumLen[i - 1];
                double t = segLen == 0 ? 0 : (s - cumLen[i - 1]) / segLen;
                Vec3 a = path.get(i - 1), b = path.get(i);
                return new Vec3(a.x + t * (b.x - a.x),
                        a.y + t * (b.y - a.y),
                        a.z + t * (b.z - a.z));
            }
        }
        return path.get(path.size() - 1);
    }
}