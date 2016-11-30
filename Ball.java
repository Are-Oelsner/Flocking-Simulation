import java.util.InputMismatchException;
import java.util.Scanner;

/// @brief A class encapsulating a state for our simulation
///
/// A Ball is the State of the simulator. It contains an intrinsic mass. It
/// contains position and velocity data. It contains operations to access, modify,
/// and integrate the state.
public class Ball {
  private Vector pos, vel;   ///< Position/velocity
  private double mass;       ///< Mass constant
  private double drag;       ///< Drag constant
  private double elasticity; ///< Elasticity constant
  private double friction;   ///< Friction constant
  private boolean rest;      ///< Simulate agent (true)

  /// @brief Copy constructor
  /// @param a Ball
  public Ball(Ball a) {
    pos = new Vector(a.pos);
    vel = new Vector(a.vel);
    mass = a.mass;
    drag = a.drag;
    elasticity = a.elasticity;
    friction = a.friction;
    rest = a.rest;
  }
  /// @brief Constructor from Scanner
  /// @param s Scanner
  public Ball(Scanner s) {
    rest = false;
    boolean done = false;
    while(s.hasNextLine() && !done) {
      String line = s.nextLine();
      Scanner l = new Scanner(line);
      String tag = l.next();
      switch(tag) {
        case "Mass":
          System.out.println("Reading mass");
          mass = l.nextDouble();
          break;
        case "Drag":
          System.out.println("Reading drag");
          drag = l.nextDouble();
          break;
        case "Elasticity":
          System.out.println("Reading elasticity");
          elasticity = l.nextDouble();
          break;
        case "Friction":
          System.out.println("Reading friction");
          friction = l.nextDouble();
          break;
        case "Position":
          System.out.println("Reading position");
          pos = new Vector(l);
          break;
        case "Velocity":
          System.out.println("Reading velocity");
          vel = new Vector(l);
          break;
        case "~Ball":
          System.out.println("Reading ~Ball");
          done = true;
          break;
        default:
          throw new InputMismatchException("Unknown tag " + tag);
      }
      l.close();
    }
  }

  /*
     public void readAgents(Scanner s) throws FileNotFoundException {
     int N = s.nextInt();
     for(int i = 0; i < N; i++) {
     Ball[i] = new Ball(s);
     }
     } */

  /// @return Mass of the ball
  public double mass() {return mass;}
  /// @return Drag of the ball
  public double drag() {return drag;}
  /// @return Elasticity of the ball
  public double elasticity() {return elasticity;}
  /// @return Friction of the ball
  public double friction() {return friction;}
  /// @return If the ball is resting
  public boolean rest() {return rest;}
  /// @return Set resting state of ball
  public void rest(boolean r) {rest = r;}
  /// @return Position of the ball
  public Vector pos() {return pos;}
  /// @return Velocity of the ball
  public Vector vel() {return vel;}
  /// @brief Set position of the ball
  /// @param p Position
  public void pos(Vector p) {pos = new Vector(p);}
  /// @brief Set velocity of the ball
  /// @param v Velocity
  public void vel(Vector v) {vel = new Vector(v);}

  /// @brief Apply force to the ball for a small time step using Euler
  ///        Integration
  /// @param force Force to apply
  /// @param t Time step
  ///
  /// x = x + t*v;
  /// v = v + t*a;
  public void applyForce(Vector force, double t) {
    Vector acc = force.multiply(1./mass); //F = m*a meaning a = F/m
    pos.addeq(vel.multiply(t));
    vel.addeq(acc.multiply(t));
  }

  /// @brief Draw the ball as a circle
  public void draw() {
    GUI.drawCircle(pos.x(), pos.y(), 2);
  }
}
