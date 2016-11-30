import java.util.InputMismatchException;
import java.util.Scanner;
import java.io.FileNotFoundException;

/// @brief Simulator for a ball.
///
/// The simulator owns the ball and determines the overall forces for an object.
/// It also determines the simulation loop of the code - clear, update, draw.
public class Simulator {

  private double timeStep; /// Timestep - time passed in virtual world per frame
  //private Ball ball;       /// Ball to simulate
  private static int numBall;
  private Ball[] ball;
  private Vector gravity;  /// Gravity force
  private Vector wind;     /// Wind force
  private Obstacle obstacle;
  private static int numObstacle;
  //private Obstacle[] obstacle;

  private static final int SAMPLES = 100;

  /// @brief Constructor from Scanner
  /// @param s Scanner
  public Simulator(Scanner s) throws FileNotFoundException {
    while(s.hasNextLine()) {
      String line = s.nextLine();
      Scanner l = new Scanner(line);
      String tag = l.next();
      switch(tag) {
        case "TimeStep":
          System.out.println("Reading timestep");
          timeStep = l.nextDouble();
          break;
        case "Gravity":
          System.out.println("Reading gravity");
          gravity = new Vector(l);
          break;
        case "Wind":
          System.out.println("Reading wind");
          wind = new Vector(l);
          break;
        case "Ball":
          System.out.println("Reading agent");
          numBall = l.nextInt();
          ball = Ball.generateBallArray(numBall);
          break;
        case "Obstacle":
          System.out.println("Reading Obstacle");
          obstacle = new Obstacle(s);
          /*numObstacle = l.nextInt();
            System.out.println("Finished Reading Obstacles");
            obstacle = Obstacle.readObstacles(s, numObstacle);
            for(int i = 0; i < numObstacle; i++) {
            System.out.println(obstacle[i]);
            }*/
          break;
        default:
          throw new InputMismatchException("Unknown tag " + tag);
      }
      l.close();
    }
  }

  /// @brief Simulation loop
  ///
  /// Clear, update, draw
  public void simulate() {
    long updateTimes[] = new long[SAMPLES], drawTimes[] = new long[SAMPLES];
    long frame = 0;
    long updateSum = 0, drawSum = 0;

    while(true) {
      int i = (int)(frame%SAMPLES);

      //Update
      long startUpdate = System.nanoTime();
      update();
      long endUpdate = System.nanoTime();
      updateSum -= updateTimes[i];
      updateTimes[i] = endUpdate - startUpdate;
      updateSum += updateTimes[i];

      //Draw
      long startDraw = System.nanoTime();

      GUI.clear();
      draw();
      drawFrameRate(updateSum, drawSum);

      GUI.show();

      long endDraw = System.nanoTime();
      drawSum -= drawTimes[i];
      drawTimes[i] = endDraw - startDraw;
      drawSum += drawTimes[i];

      frame++;
    }
  }

  /// @brief Update the position of the ball
  private void update() {
    for(int i = 0; i < numBall; i++) {

      if(ball[i].rest()) {
        System.out.printf("Ball %d at rest", i);
        return;
      }

      double tr = timeStep;
      while(tr > 0) {
        Ball ballnew = new Ball(ball[i]);
        Vector force = determineForces(ball[i]);
        ballnew.applyForce(force, tr);
        System.out.println("Ball " + i);
        System.out.println(ballnew.pos());

        Collision c = checkCollision(ball[i].pos(), ballnew.pos());
        if(c != null) {
          ball[i].applyForce(force, tr*c.f());
          resolveCollision(c, ball[i]);
          tr -= tr*c.f();
        }
        else {
          ball[i] = ballnew;
          tr = 0;
        }
      }
    }
  }

  /// @brief Determine forces on the ball
  private Vector determineForces(Ball ball) {
    Vector fg = gravity.multiply(ball.mass()); //F_g = m*g where g is acceleration due to gravity
    Vector fw = wind.multiply(ball.drag());    //F_w = d*w where w is wind vector
    Vector fa = ball.vel().multiply(-ball.drag()); //F_a = -d*v where v is velocity of ball
    System.out.println("fa " + fa);
    System.out.println("velocity " + ball.vel());
    System.out.println("position " + ball.pos());

    //F_tot = F_g + F_w + F_a
    return fg.add(fw.add(fa));
  }


  /// @brief Collision check linear motion of ball between two positions
  /// @return First collision
  private Collision checkCollision(Vector p, Vector pnew) {

    Collision c = new Collision(Double.POSITIVE_INFINITY, null);
    double f = Double.POSITIVE_INFINITY;
    if(pnew.x() > 50) {
      f = (50-p.x())/(pnew.x()-p.x());
      c = new Collision(f, new Vector(-1, 0));
    }
    else if(pnew.x() < -50) {
      f = (-50-p.x())/(pnew.x()-p.x());
      if(f < c.f())
        c = new Collision(f, new Vector(1, 0));
    }
    if(pnew.y() > 50) {
      f = (50-p.y())/(pnew.y()-p.y());
      if(f < c.f())
        c = new Collision(f, new Vector(0, -1));
    }
    else if(pnew.y() < -50) {
      f = (-50-p.y())/(pnew.y()-p.y());
      if(f < c.f())
        c = new Collision(f, new Vector(0, 1));
    }
    /* Collision temp;
       for(int i = 0; i < numObstacle; i++) {
       temp = checkCollisionWithObstacle(obstacle, p, pnew);
       if(temp.f() < f) {
       c = temp;
       f = temp.f();
       }
    // } */
    if(c.f() != Double.POSITIVE_INFINITY) {
      return c;
    }
    else
      return null;
  }

  private static Collision checkCollisionWithObstacle(Obstacle obs, Vector p, Vector r) {
    int N = obs.getNumPoints();
    Collision temp;
    Collision c = new Collision();
    double firstCollision = Double.POSITIVE_INFINITY;
    for(int i = 0; i < N; i++) {
      temp = checkIntersection(p, r, obs.getEdge(i)[0], obs.getEdge(i)[1]);
      if(temp.f() < firstCollision) {
        firstCollision = temp.f();
        c = temp;
      }
    }
    return c;
  }

  //Got help from http://stackoverflow.com/questions/563198/how-do-you-
  //detect-where-two-line-segments-intersect
  private static Collision checkIntersection(
      Vector p, Vector r, Vector q, Vector s) {
    Collision c  = new Collision(Double.POSITIVE_INFINITY,null);
    r.subeq(p);
    s.subeq(q);

    Vector qMinusP = q.sub(p);
    double rCrossS = r.cross(s);

    double t = qMinusP.cross(s)/rCrossS;
    double u = qMinusP.cross(r)/rCrossS;

    if(equals(rCrossS, 0)) {
      System.out.println("No Collision detected");
      return c;
    }
    else if(t>=0 && t<=1 && u>=0 && u<=1) {
      c = new Collision(t, p.add(r.multiply(t)));
      System.out.println("Calculating intersection");
      return c;
    }
    else {
      return c;
    }
      }


  public static boolean equals(double a, double b) {
    double Epsilon = 1e-15;
    if(Math.abs(a - b) < Epsilon) {
      return true;
    }
    else { return false; }
  }

  private void resolveCollision(Collision c, Ball ball) {
    Vector vn = c.n().multiply(c.n().dot(ball.vel()));
    Vector vt = ball.vel().sub(vn);
    vn.multiplyeq(-ball.elasticity());
    vt.multiplyeq(1-ball.friction());
    ball.vel(vn.add(vt));

    //TODO remove hard constant for resting distance
    if(ball.vel().magnitude() < 1)
      ball.rest(true);
  }

  //My version of collision resolution code
  /*
     private void resolveCollisionAre(Collision c) {
     Vector scaledVel = ball.vel().multiply(c.f());
     Vector N = scaledVel.dot(c.n());
     Vector T = scaledVel + N;
     N.multiplyeq(-ball.elasticity());
     T.multiplyeq(1-ball.friction());
     ball.vel(V.add(T));
     } */

  private void draw() {
    for(int i = 0; i < numBall; i++) {
      ball[i].draw();
    }
    obstacle.draw();
    //drawObstacleArray(obstacle);
  }

  private void drawObstacleArray(Obstacle[] obs) {
    for(int i = 0; i < numObstacle; i++) {
      obs[i].draw();
    }
  }

  private void drawFrameRate(long updateSum, long drawSum) {
    double updateTime = updateSum / SAMPLES / 1e9;
    double drawTime = drawSum / SAMPLES / 1e9;
    double frameTime = updateTime + drawTime;

    GUI.text(-51, 53, String.format("My Flocking Simulation"));
    GUI.text(-51, 50, String.format("Update Time: %5.3f", updateTime));
    GUI.text(-51, 47, String.format("  Draw Time: %5.3f", drawTime));
    GUI.text(-51, 44, String.format(" Frame Time: %5.3f", frameTime));
    GUI.text(-51, 41, String.format(" Frame Rate: %5.2f", Math.min(1./frameTime, 1/0.016)));
  }

  private static String toString(Collision c) {
    return String.valueOf(c.f()) + " " + c.n().toString();}

  public static int getNumObstacles() {return numObstacle;}

  public static void main(String[] args) {
    double a1x = Double.parseDouble(args[0]);
    double a1y = Double.parseDouble(args[1]);
    double a2x = Double.parseDouble(args[2]);
    double a2y = Double.parseDouble(args[3]);

    double b1x = Double.parseDouble(args[4]);
    double b1y = Double.parseDouble(args[5]);
    double b2x = Double.parseDouble(args[6]);
    double b2y = Double.parseDouble(args[7]);

    Vector p = new Vector(a1x, a1y);
    Vector r = new Vector(a2x, a2y);
    Vector q = new Vector(b1x, b1y);
    Vector s = new Vector(b2x, b2y);

    Collision c = new Collision();

    //Active test for Intersection check function
    c = checkIntersection(p, r, q, s);
    System.out.println(toString(c));
  }
}
