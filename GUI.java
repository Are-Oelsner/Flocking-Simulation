/// DID NOT COMMENT THIS CODE. BEWARE!

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.*;

import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

public class GUI implements KeyListener {

    //size
    private static int width = 600;
    private static int height = 600;
    private static double xmin = -55, xmax = 55, ymin = -55, ymax = 55;

    //singleton for callbacks
    public static GUI jma = new GUI();

    // double buffered graphics
    private static BufferedImage offscreenImage, onscreenImage;
    private static Graphics2D offscreen, onscreen;

    // the frame for drawing to the screen
    private static JFrame frame;

    private static long nextDraw = -1;

    private static final Font DEFAULT_FONT = new Font("Monospaced", Font.BOLD, 12);

    private GUI() {
    }

    static {
        Init();
    }

    private static void Init() {
        frame = new JFrame();
        offscreenImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        onscreenImage  = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        offscreen = offscreenImage.createGraphics();
        onscreen  = onscreenImage.createGraphics();
        clear();

        ImageIcon icon = new ImageIcon(onscreenImage);
        JLabel draw = new JLabel(icon);

        frame.setContentPane(draw);
        frame.addKeyListener(jma);    // JLabel cannot get keyboard focus
        frame.setResizable(false);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);            // closes all windows
        //frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);      // closes only current window
        frame.setTitle("Java Motion Applet");
        //frame.setJMenuBar(createMenuBar());
        frame.pack();
        frame.requestFocusInWindow();
        frame.setVisible(true);
    }

    public static void clear() {
        offscreen.setColor(Color.black);
        offscreen.fillRect(0, 0, width, height);
        //offscreen.setColor(Color.red);
        //drawCircle(0, 0, 2);
        offscreen.setColor(Color.white);
        offscreen.draw(new Rectangle2D.Double(scaleX(-52), scaleY(52), factorX(104), factorY(104)));
    }

    public static void pause(int t) {
        // sleep until the next time we're allowed to draw
        long millis = System.currentTimeMillis();
        if (millis < nextDraw) {
            try {
                Thread.sleep(nextDraw - millis);
            }
            catch (InterruptedException e) {
                System.out.println("Error sleeping");
            }
            millis = nextDraw;
        }

        // when are we allowed to draw again
        nextDraw = millis + t;
    }

    public static void show() {
        onscreen.drawImage(offscreenImage, 0, 0, null);
        frame.repaint();
        pause(16); //16
    }

    public void keyTyped(KeyEvent e) {
        //do nothing: Java is dumb.
    }

    public void keyPressed(KeyEvent e) {
        switch(e.getKeyCode()) {
            default: break;
        }
    }

    public void keyReleased(KeyEvent e) {
        switch(e.getKeyCode()) {
            default: break;
        }
    }

    private static double scaleX(double x) { return (width  * (x - xmin) / (xmax - xmin)); }
    private static double scaleY(double y) { return (height * (ymax - y) / (ymax - ymin)); }
    private static double factorX(double w) { return (w * width  / Math.abs(xmax - xmin));  }
    private static double factorY(double h) { return (h * height / Math.abs(ymax - ymin));  }

    //Draw the ball at center (x, y) with radius r
    public static void drawCircle(double x, double y, double r) {
        double rx = factorX(2*r);
        double ry = factorY(2*r);
        double N = Math.random()*6;
        Color color;
        if(N <= 1) {color = Color.BLUE;}
        else if(N > 1 && N <= 2) {color = Color.RED;}
        else if(N > 2 && N <= 3) {color = Color.YELLOW;}
        else if(N > 3 && N <= 4) {color = Color.GREEN;}
        else if(N > 4 && N <= 5) {color = Color.CYAN;}
        else {color = Color.PINK;}
        offscreen.setColor(color);
        offscreen.fill(new Ellipse2D.Double(
                    scaleX(x) - rx/2, scaleY(y) - ry/2, rx, ry));
    }

    public static void drawPolygon(double[] x, double[] y, int numPoints) {
        int[] _x = new int[numPoints];
        int[] _y = new int[numPoints];
        for(int i = 0; i < numPoints; i++) {
            _x[i] = (int)scaleX(x[i]);
            _y[i] = (int)scaleY(y[i]);
        }
        offscreen.drawPolygon(_x, _y, numPoints);
        //drawPolygon(x, y, numPoints);
    }



    /// @brief Write text in the current font, left-aligned at (x, y).
    /// @param  x the x-coordinate of the text
    /// @param  y the y-coordinate of the text
    /// @param  text the text
    public static void text(double x, double y, String text) {
        if (text == null)
            throw new NullPointerException();
        offscreen.setColor(Color.WHITE);
        offscreen.setFont(DEFAULT_FONT);
        FontMetrics metrics = offscreen.getFontMetrics();
        double xs = scaleX(x);
        double ys = scaleY(y);
        int hs = metrics.getDescent();
        offscreen.drawString(text, (float) xs, (float) (ys + hs));
    }

}
