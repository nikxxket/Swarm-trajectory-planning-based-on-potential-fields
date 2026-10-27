package logic;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;

public class World3D {
    public int W, H, Z;
    public boolean[][][] obstacle;
    double[][][] danger;            // виден только из logic (ThetaStar3D)
    public double[][] dangerColumn;
    double[][][] potential;         // не нужен снаружи
    double dangerMax = 1.0;

    static final double P0 = 4.0;
    static final double MU = 200.0;
    static final double LAMBDA = 0.5;

    public List<Drone3D> drones = new ArrayList<>();
    public int[] goal = {25, 14, 4};

    public World3D(int w, int h, int z) {
        W = w; H = h; Z = z;
        obstacle = new boolean[W][H][Z];
        danger = new double[W][H][Z];
        potential = new double[W][H][Z];
        dangerColumn = new double[W][H];
    }

    public void buildRooms() {
        for (int z = 0; z < Z; z++) {
            for (int x = 0; x < W; x++) { obstacle[x][0][z] = true; obstacle[x][H - 1][z] = true; }
            for (int y = 0; y < H; y++) { obstacle[0][y][z] = true; obstacle[W - 1][y][z] = true; }
        }
        for (int y = 1; y < H - 1; y++) {
            if (y >= 13 && y <= 18) continue;
            for (int z = 0; z < Z; z++) obstacle[10][y][z] = true;
        }
        for (int y = 1; y < H - 1; y++) {
            if ((y >= 5 && y <= 9) || (y >= 20 && y <= 24)) continue;
            for (int z = 0; z < Z; z++) obstacle[20][y][z] = true;
        }
        addBlock(4, 8, 0, 3, 3, 2);
        addBlock(13, 10, 0, 4, 3, 2);
        addBlock(23, 8, 5, 3, 3, 2);
        addBlock(24, 20, 5, 4, 3, 2);
        addBlock(6, 20, 0, 1, 1, Z);
        addBlock(14, 20, 0, 1, 1, Z);
        obstacle[goal[0]][goal[1]][goal[2]] = false;
    }

    void addBlock(int x0, int y0, int z0, int sx, int sy, int sz) {
        for (int x = x0; x < Math.min(W, x0 + sx); x++)
            for (int y = y0; y < Math.min(H, y0 + sy); y++)
                for (int z = z0; z < Math.min(Z, z0 + sz); z++)
                    obstacle[x][y][z] = true;
    }

    public void spawnDrones() {
        drones.clear();
        Color[] colors = {
                new Color(30, 100, 220),
                new Color(20, 160, 60),
                new Color(220, 130, 20),
                new Color(160, 40, 160)
        };
        double[] speeds = {0.22, 0.18, 0.25, 0.20};
        int[][] starts = {{3, 3, 4}, {3, 26, 4}, {15, 3, 4}, {15, 26, 4}};
        for (int i = 0; i < starts.length; i++) {
            int x = starts[i][0], y = starts[i][1], z = starts[i][2];
            while (x < W && obstacle[x][y][z]) x++;
            drones.add(new Drone3D(x, y, z, goal, i, colors[i], speeds[i]));
        }
    }

    public void shuffleDrones() {
        List<int[]> free = new ArrayList<>();
        for (int x = 2; x < W - 2; x++)
            for (int y = 2; y < H - 2; y++)
                for (int z = 1; z < Z - 1; z++) {
                    if (obstacle[x][y][z]) continue;
                    double gx = x - goal[0], gy = y - goal[1], gz = z - goal[2];
                    if (Math.sqrt(gx * gx + gy * gy + gz * gz) < 5) continue;
                    free.add(new int[]{x, y, z});
                }
        Collections.shuffle(free);

        int minDist = 5;
        int assigned = 0;
        for (int[] pos : free) {
            if (assigned >= drones.size()) break;
            boolean ok = true;
            for (int j = 0; j < assigned; j++) {
                Drone3D d = drones.get(j);
                double dx = pos[0] - d.x, dy = pos[1] - d.y, dz = pos[2] - d.z;
                if (Math.sqrt(dx * dx + dy * dy + dz * dz) < minDist) { ok = false; break; }
            }
            if (!ok) continue;
            Drone3D d = drones.get(assigned++);
            d.x = pos[0]; d.y = pos[1]; d.z = pos[2];
            d.progress = 0;
        }
        for (int i = assigned; i < drones.size(); i++) {
            int[] pos = free.get(i);
            Drone3D d = drones.get(i);
            d.x = pos[0]; d.y = pos[1]; d.z = pos[2];
            d.progress = 0;
        }
    }

    public void computeDanger() {
        int R = (int) Math.ceil(P0);

        double[][][] table = new double[2 * R + 1][2 * R + 1][2 * R + 1];
        for (int dx = -R; dx <= R; dx++)
            for (int dy = -R; dy <= R; dy++)
                for (int dz = -R; dz <= R; dz++) {
                    double p = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (p < 0.5) p = 0.5;
                    table[R + dx][R + dy][R + dz] = (p < P0)
                            ? 0.5 * MU * Math.pow(1.0 / p - 1.0 / P0, 2) : 0;
                }

        IntStream.range(0, W).parallel().forEach(x -> {
            for (int y = 0; y < H; y++) {
                double colMax = 0;
                for (int z = 0; z < Z; z++) {
                    if (obstacle[x][y][z]) {
                        danger[x][y][z] = Double.NaN;
                        potential[x][y][z] = 1e9;
                        continue;
                    }
                    double Urep = 0;
                    int xlo = Math.max(0, x - R), xhi = Math.min(W - 1, x + R);
                    int ylo = Math.max(0, y - R), yhi = Math.min(H - 1, y + R);
                    int zlo = Math.max(0, z - R), zhi = Math.min(Z - 1, z + R);
                    for (int nx = xlo; nx <= xhi; nx++)
                        for (int ny = ylo; ny <= yhi; ny++)
                            for (int nz = zlo; nz <= zhi; nz++)
                                if (obstacle[nx][ny][nz])
                                    Urep += table[R + (nx - x)][R + (ny - y)][R + (nz - z)];

                    double dx = x - goal[0], dy = y - goal[1], dz = z - goal[2];
                    double dg = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    double Uatt = 0.5 * LAMBDA * dg * dg;

                    danger[x][y][z] = Urep;
                    potential[x][y][z] = Uatt + Urep;
                    if (Urep > colMax) colMax = Urep;
                }
                dangerColumn[x][y] = colMax;
            }
        });

        for (int x = 0; x < W; x++)
            for (int y = 0; y < H; y++)
                if (dangerColumn[x][y] > dangerMax) dangerMax = dangerColumn[x][y];
    }
}