import java.awt.BorderLayout;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import logic.World3D;
import visualization.MapPanel3D;

public class ThetaStarAPFDemo3D extends JFrame {

    public ThetaStarAPFDemo3D() {
        setTitle("Theta*-APF");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setSize(1200, 950);
        setLocationRelativeTo(null);

        World3D world = new World3D(30, 30, 8);
        world.buildRooms();
        world.computeDanger();
        world.spawnDrones();

        MapPanel3D panel = new MapPanel3D(world);

        JButton playBtn = new JButton("Пуск / Пауза");
        playBtn.addActionListener(e -> panel.toggleAnimation());

        JButton resetBtn = new JButton("Перезапустить");
        resetBtn.addActionListener(e -> { panel.resetAnimation(); panel.repaint(); });

        JButton shuffleBtn = new JButton("Перемешать");
        shuffleBtn.addActionListener(e -> {
            panel.stopAnimation();
            long t0 = System.currentTimeMillis();
            world.shuffleDrones();
            panel.computePaths();
            long t1 = System.currentTimeMillis();
            System.out.println("Перемешать: " + (t1 - t0) + " мс");
            panel.startAnimation();
            panel.repaint();
        });

        JPanel ctrl = new JPanel();
        ctrl.add(playBtn);
        ctrl.add(resetBtn);
        ctrl.add(shuffleBtn);
        add(ctrl, BorderLayout.SOUTH);
        add(panel, BorderLayout.CENTER);

        long t0 = System.currentTimeMillis();
        panel.computePaths();
        long t1 = System.currentTimeMillis();
        System.out.println("Первое планирование: " + (t1 - t0) + " мс");
        panel.startAnimation();
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new ThetaStarAPFDemo3D().setVisible(true));
    }
}