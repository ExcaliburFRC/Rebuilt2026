package frc.robot.util;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

/**
 * Simple Swing window that captures WASD + QE keys and exposes axis values.
 * Used only in simulation as a keyboard fallback for driving.
 */
public class SimKeyboard {
    private static volatile boolean w, a, s, d, q, e;

    public static void startIfNeeded() {
        // Start UI on EDT
        SwingUtilities.invokeLater(() -> {
            JFrame f = new JFrame("Sim Keyboard - WASD + Q/E");
            f.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
            f.setSize(300, 150);
            f.setLayout(new BorderLayout());

            JLabel info = new JLabel("W/S: forward/back, A/D: strafe, Q/E: rotate", SwingConstants.CENTER);
            info.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
            f.add(info, BorderLayout.CENTER);

            f.addKeyListener(new KeyAdapter() {
                @Override
                public void keyPressed(KeyEvent ke) {
                    switch (ke.getKeyCode()) {
                        case KeyEvent.VK_W -> w = true;
                        case KeyEvent.VK_A -> a = true;
                        case KeyEvent.VK_S -> s = true;
                        case KeyEvent.VK_D -> d = true;
                        case KeyEvent.VK_Q -> q = true;
                        case KeyEvent.VK_E -> e = true;
                    }
                }

                @Override
                public void keyReleased(KeyEvent ke) {
                    switch (ke.getKeyCode()) {
                        case KeyEvent.VK_W -> w = false;
                        case KeyEvent.VK_A -> a = false;
                        case KeyEvent.VK_S -> s = false;
                        case KeyEvent.VK_D -> d = false;
                        case KeyEvent.VK_Q -> q = false;
                        case KeyEvent.VK_E -> e = false;
                    }
                }
            });

            // Ensure the frame can receive key events
            f.setFocusable(true);
            f.setVisible(true);
            f.requestFocus();
        });
    }

    // Axis values in range [-1,1]
    public static double getForward() {
        double v = 0;
        if (w) v += 1.0;
        if (s) v -= 1.0;
        return clamp(v);
    }

    public static double getStrafe() {
        double v = 0;
        if (d) v += 1.0;
        if (a) v -= 1.0;
        return clamp(v);
    }

    public static double getRotate() {
        double v = 0;
        if (e) v += 1.0;
        if (q) v -= 1.0;
        return clamp(v);
    }

    private static double clamp(double v) {
        if (v > 1) return 1;
        if (v < -1) return -1;
        return v;
    }
}
