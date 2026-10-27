package visualization;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.Timer;

import logic.Drone3D;
import logic.ThetaStar3D;
import logic.Vec3;
import logic.World3D;

public class MapPanel3D extends JPanel {

    World3D world;
    Camera3D camera;
    Timer timer;
    long tick = 0;

    int dragX, dragY;
    boolean dragging = false;
    boolean showDanger = true;

    List<int[]> visibleCubesCache = null;
    List<int[]> floorTilesCache = null;

    public MapPanel3D(World3D world) {
        this.world = world;
        this.camera = new Camera3D(new Vec3(world.W / 2.0, world.H / 2.0, world.Z / 3.0));
        setBackground(Color.WHITE);
        setPreferredSize(new Dimension(900, 800));
        setFocusable(true);

        timer = new Timer(16, e -> {
            boolean any = false;
            for (Drone3D d : world.drones) if (d.advance()) any = true;
            tick++;
            repaint();
            if (!any) timer.stop();
        });

        addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                dragging = true; dragX = e.getX(); dragY = e.getY();
            }
            @Override public void mouseReleased(MouseEvent e) { dragging = false; }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override public void mouseDragged(MouseEvent e) {
                if (!dragging) return;
                int dx = e.getX() - dragX, dy = e.getY() - dragY;
                dragX = e.getX(); dragY = e.getY();
                camera.azimuth += dx * 0.01;
                camera.elevation = Math.max(Math.toRadians(5),
                        Math.min(Math.toRadians(85), camera.elevation + dy * 0.01));
                camera.update();
                repaint();
            }
        });
        addMouseWheelListener(e -> {
            camera.distance *= (1 + e.getWheelRotation() * 0.1);
            camera.distance = Math.max(10, Math.min(150, camera.distance));
            camera.update();
            repaint();
        });
    }

    public void computePaths() {
        ThetaStar3D planner = new ThetaStar3D(world);
        world.drones.parallelStream().forEach(d -> {
            d.path = planner.search(d.x, d.y, d.z, d.goal);
            d.initPath();
        });
    }

    public void startAnimation() {
        for (Drone3D d : world.drones) if (!d.finished()) { timer.start(); return; }
    }
    public void stopAnimation()   { timer.stop(); }
    public void toggleAnimation() { if (timer.isRunning()) timer.stop(); else startAnimation(); }
    public void resetAnimation() {
        timer.stop();
        for (Drone3D d : world.drones) d.progress = 0;
        tick = 0;
        startAnimation();
    }

    private void buildCacheIfNeeded() {
        if (visibleCubesCache != null && floorTilesCache != null) return;

        List<int[]> floors = new ArrayList<>();
        for (int x = 0; x < world.W; x++)
            for (int y = 0; y < world.H; y++)
                if (!world.obstacle[x][y][0]) floors.add(new int[]{x, y});
        floorTilesCache = floors;

        List<int[]> cubes = new ArrayList<>();
        for (int x = 0; x < world.W; x++)
            for (int y = 0; y < world.H; y++)
                for (int z = 0; z < world.Z; z++)
                    if (world.obstacle[x][y][z] && isVisibleCube(x, y, z))
                        cubes.add(new int[]{x, y, z});
        visibleCubesCache = cubes;
    }

    boolean isVisibleCube(int x, int y, int z) {
        if (x == 0 || x == world.W - 1 || y == 0 || y == world.H - 1
                || z == 0 || z == world.Z - 1) return true;
        return !world.obstacle[x - 1][y][z] || !world.obstacle[x + 1][y][z]
                || !world.obstacle[x][y - 1][z] || !world.obstacle[x][y + 1][z]
                || !world.obstacle[x][y][z - 1] || !world.obstacle[x][y][z + 1];
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setColor(Color.WHITE);
        g2.fillRect(0, 0, getWidth(), getHeight());
        camera.update();

        buildCacheIfNeeded();

        List<int[]> floorTiles = new ArrayList<>(floorTilesCache);
        floorTiles.sort((a, b) -> Double.compare(
                camera.depthOf(new Vec3(b[0] + 0.5, b[1] + 0.5, 0)),
                camera.depthOf(new Vec3(a[0] + 0.5, a[1] + 0.5, 0))));
        for (int[] t : floorTiles) drawFloorTile(g2, t[0], t[1]);

        List<int[]> cubes = new ArrayList<>(visibleCubesCache);
        cubes.sort((a, b) -> Double.compare(
                camera.depthOf(new Vec3(b[0] + 0.5, b[1] + 0.5, b[2] + 0.5)),
                camera.depthOf(new Vec3(a[0] + 0.5, a[1] + 0.5, a[2] + 0.5))));
        for (int[] c : cubes) drawCube(g2, c[0], c[1], c[2]);

        drawPaths(g2);
        drawGoal(g2);
        drawDrones(g2);
    }

    void drawFloorTile(Graphics2D g2, int x, int y) {
        Color c;
        if (showDanger) {
            double t = Math.sqrt(Math.min(1.0, world.dangerColumn[x][y] / 20.0));
            c = dangerColor(t);
        } else {
            c = new Color(235, 235, 240);
        }
        Vec3[] corners = {
                new Vec3(x, y, 0), new Vec3(x + 1, y, 0),
                new Vec3(x + 1, y + 1, 0), new Vec3(x, y + 1, 0)
        };
        Polygon p = new Polygon();
        for (Vec3 v : corners) {
            Point2D.Double s = camera.project(v, getWidth(), getHeight());
            if (s == null) return;
            p.addPoint((int) s.x, (int) s.y);
        }
        g2.setColor(c);
        g2.fillPolygon(p);
        g2.setColor(new Color(0, 0, 0, 25));
        g2.setStroke(new BasicStroke(0.6f));
        g2.drawPolygon(p);
    }

    void drawCube(Graphics2D g2, int x, int y, int z) {
        Vec3[] v = new Vec3[8];
        for (int i = 0; i < 8; i++)
            v[i] = new Vec3(x + (i & 1), y + ((i >> 1) & 1), z + ((i >> 2) & 1));

        int[][] top = {{4, 5, 6, 7}};
        int[][] sides = {{1, 5, 7, 3}, {2, 6, 7, 3}, {0, 4, 6, 2}, {0, 4, 5, 1}};

        Vec3 camPos = camera.position;
        Vec3 cubeCenter = new Vec3(x + 0.5, y + 0.5, z + 0.5);
        Vec3 toCam = camPos.sub(cubeCenter).norm();

        Color base = new Color(70, 70, 80);
        Color light = base.brighter();

        if (toCam.z > 0.05) drawQuad(g2, v, top[0], light);
        double[] nx = {1, 0, -1, 0};
        double[] ny = {0, 1, 0, -1};
        for (int i = 0; i < 4; i++) {
            if (toCam.x * nx[i] + toCam.y * ny[i] > 0.05) {
                Color side = base;
                if (i == 0 || i == 1) side = base.darker();
                drawQuad(g2, v, sides[i], side);
            }
        }
    }

    void drawQuad(Graphics2D g2, Vec3[] v, int[] idx, Color c) {
        Polygon p = new Polygon();
        for (int i : idx) {
            Point2D.Double s = camera.project(v[i], getWidth(), getHeight());
            if (s == null) return;
            p.addPoint((int) s.x, (int) s.y);
        }
        g2.setColor(c);
        g2.fillPolygon(p);
        g2.setColor(c.darker());
        g2.setStroke(new BasicStroke(0.7f));
        g2.drawPolygon(p);
    }

    void drawPaths(Graphics2D g2) {
        for (Drone3D d : world.drones) {
            if (d.path == null || d.path.size() < 2) continue;
            g2.setColor(new Color(d.color.getRed(), d.color.getGreen(), d.color.getBlue(), 180));
            g2.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            Point2D.Double prev = null;
            for (Vec3 p : d.path) {
                Point2D.Double s = camera.project(p, getWidth(), getHeight());
                if (s != null && prev != null) g2.drawLine((int) prev.x, (int) prev.y, (int) s.x, (int) s.y);
                prev = s;
            }
        }
    }

    void drawGoal(Graphics2D g2) {
        Vec3 g = new Vec3(world.goal[0] + 0.5, world.goal[1] + 0.5, world.goal[2] + 0.5);
        Point2D.Double s = camera.project(g, getWidth(), getHeight());
        if (s == null) return;
        int r = 14;
        g2.setColor(new Color(255, 60, 60, 90));
        g2.fillOval((int) s.x - r - 4, (int) s.y - r - 4, (r + 4) * 2, (r + 4) * 2);
        g2.setColor(new Color(220, 30, 30));
        g2.fillOval((int) s.x - r / 2, (int) s.y - r / 2, r, r);
        g2.setColor(Color.WHITE);
        g2.setStroke(new BasicStroke(1.5f));
        g2.drawOval((int) s.x - r / 2, (int) s.y - r / 2, r, r);
    }

    void drawDrones(Graphics2D g2) {
        for (Drone3D d : world.drones) {
            Vec3 pos = d.getPosition();
            Point2D.Double s = camera.project(pos, getWidth(), getHeight());
            if (s == null) continue;
            int r = 14;
            Vec3 shadow = new Vec3(pos.x, pos.y, 0);
            Point2D.Double sh = camera.project(shadow, getWidth(), getHeight());
            if (sh != null) {
                g2.setColor(new Color(0, 0, 0, 70));
                g2.fillOval((int) sh.x - 7, (int) sh.y - 3, 14, 6);
            }
            g2.setColor(d.color);
            g2.fillOval((int) s.x - r / 2, (int) s.y - r / 2, r, r);
            g2.setColor(Color.WHITE);
            g2.setStroke(new BasicStroke(1.5f));
            g2.drawOval((int) s.x - r / 2, (int) s.y - r / 2, r, r);
            g2.setColor(Color.WHITE);
            g2.setFont(new Font("SansSerif", Font.BOLD, 11));
            String str = String.valueOf(d.id);
            FontMetrics fm = g2.getFontMetrics();
            g2.drawString(str, (int) s.x - fm.stringWidth(str) / 2,
                    (int) s.y + fm.getAscent() / 2 - 1);
        }
    }

    Color dangerColor(double t) {
        t = Math.max(0, Math.min(1, t));
        float hue = (float) (0.66 * (1 - t));
        float sat = (float) (0.55 + 0.35 * t);
        return Color.getHSBColor(hue, sat, 0.98f);
    }
}