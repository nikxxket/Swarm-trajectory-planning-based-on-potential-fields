package logic;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class ThetaStar3D {
    World3D world;

    static final double DANGER_REF = 10.0;
    static final double DANGER_WEIGHT = 2.0;
    static final double DANGER_CAP = 3.0;

    static final int[][] DIRS;
    static {
        List<int[]> d = new ArrayList<>();
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                    if (!(dx == 0 && dy == 0 && dz == 0))
                        d.add(new int[]{dx, dy, dz});
        DIRS = d.toArray(new int[0][]);
    }

    public ThetaStar3D(World3D w) { this.world = w; }

    double segmentCost(int x0, int y0, int z0, int x1, int y1, int z1) {
        double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
        double totalDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (totalDist < 1e-9) return 0;
        int steps = Math.max(1, (int) Math.ceil(totalDist));
        double segLen = totalDist / steps;
        double cost = 0;
        for (int i = 1; i <= steps; i++) {
            double t = (double) i / steps;
            int sx = (int) Math.round(x0 + t * dx);
            int sy = (int) Math.round(y0 + t * dy);
            int sz = (int) Math.round(z0 + t * dz);
            if (sx < 0 || sy < 0 || sz < 0 || sx >= world.W || sy >= world.H || sz >= world.Z) continue;
            double u = world.danger[sx][sy][sz];
            if (Double.isNaN(u)) continue;
            double un = Math.min(u / DANGER_REF, DANGER_CAP);
            cost += segLen * (1.0 + DANGER_WEIGHT * un);
        }
        return cost;
    }

    public List<Vec3> search(int sx, int sy, int sz, int[] goal) {
        int W = world.W, H = world.H, Z = world.Z;
        Map<Long, Node3D> best = new HashMap<>();
        PriorityQueue<Node3D> open = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f));
        Set<Long> closed = new HashSet<>();

        Node3D s = new Node3D(sx, sy, sz, null, 0, heuristic(null, sx, sy, sz, goal));
        best.put(key(sx, sy, sz), s);
        open.add(s);

        while (!open.isEmpty()) {
            Node3D cur = open.poll();
            if (best.get(key(cur.x, cur.y, cur.z)) != cur) continue;
            if (closed.contains(key(cur.x, cur.y, cur.z))) continue;
            closed.add(key(cur.x, cur.y, cur.z));

            if (cur.x == goal[0] && cur.y == goal[1] && cur.z == goal[2])
                return reconstruct(cur);

            for (int[] dir : DIRS) {
                int nx = cur.x + dir[0];
                int ny = cur.y + dir[1];
                int nz = cur.z + dir[2];
                if (nx < 0 || ny < 0 || nz < 0 || nx >= W || ny >= H || nz >= Z) continue;
                if (world.obstacle[nx][ny][nz]) continue;

                Node3D parent = cur.parent;
                double newG; Node3D newParent;
                if (parent != null && lineOfSight(parent.x, parent.y, parent.z, nx, ny, nz)) {
                    newG = parent.g + segmentCost(parent.x, parent.y, parent.z, nx, ny, nz);
                    newParent = parent;
                } else {
                    newG = cur.g + segmentCost(cur.x, cur.y, cur.z, nx, ny, nz);
                    newParent = cur;
                }
                long k = key(nx, ny, nz);
                Node3D existing = best.get(k);
                if (existing == null || newG < existing.g - 1e-9) {
                    Node3D node = new Node3D(nx, ny, nz, newParent, newG,
                            heuristic(newParent, nx, ny, nz, goal));
                    best.put(k, node);
                    open.add(node);
                    closed.remove(k);
                }
            }
        }
        return null;
    }

    double heuristic(Node3D parent, int nx, int ny, int nz, int[] goal) {
        double dx = nx - goal[0], dy = ny - goal[1], dz = nz - goal[2];
        double d = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (parent == null) return d;
        double thx = goal[0] - parent.x, thy = goal[1] - parent.y, thz = goal[2] - parent.z;
        double thLen = Math.sqrt(thx * thx + thy * thy + thz * thz);
        if (thLen < 1e-9) return d;
        double alx = nx - parent.x, aly = ny - parent.y, alz = nz - parent.z;
        double alLen = Math.sqrt(alx * alx + aly * aly + alz * alz);
        if (alLen < 1e-9) return d;
        double cos = (thx * alx + thy * aly + thz * alz) / (thLen * alLen);
        return d * (1 - 0.4 * cos);
    }

    boolean lineOfSight(int x0, int y0, int z0, int x1, int y1, int z1) {
        int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0), dz = Math.abs(z1 - z0);
        int sx = Integer.compare(x1, x0), sy = Integer.compare(y1, y0), sz = Integer.compare(z1, z0);
        int cx = x0, cy = y0, cz = z0;
        while (true) {
            if (cx < 0 || cy < 0 || cz < 0 ||
                    cx >= world.W || cy >= world.H || cz >= world.Z) return false;
            if (world.obstacle[cx][cy][cz]) return false;
            if (cx == x1 && cy == y1 && cz == z1) return true;

            int ex = Math.abs(x1 - cx), ey = Math.abs(y1 - cy), ez = Math.abs(z1 - cz);
            int mx = Math.max(ex, Math.max(ey, ez));
            if (mx == ex) cx += sx;
            else if (mx == ey) cy += sy;
            else cz += sz;
        }
    }

    List<Vec3> reconstruct(Node3D n) {
        LinkedList<Vec3> path = new LinkedList<>();
        while (n != null) {
            path.addFirst(new Vec3(n.x + 0.5, n.y + 0.5, n.z + 0.5));
            n = n.parent;
        }
        return path;
    }

    long key(int x, int y, int z) {
        return ((long) (x & 0xFFFF) << 32) | ((long) (y & 0xFFFF) << 16) | (z & 0xFFFF);
    }
}