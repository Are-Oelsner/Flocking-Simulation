import java.util.InputMismatchException;
import java.util.Scanner;

/// @brief Denotes a mathematical 2d point/vector (x, y)
///
/// This class supports the following common operations of vectors:
///  - magnitude
///  - dot
///  - add
///  - subtract
///  - scalar multiply
///  - normalization (creating unit vector)
///  - Type conversion to/from String
public class Vector {
  private double x, y; ///< Positional data

  /// @brief Default constructor for (0, 0)
  public Vector() {
    x = 0;
    y = 0;
  }
  /// @brief Constructor for (x, y) of arbitrary points/vectors
  /// @param _x X coordinate
  /// @param _y Y coordinate
  public Vector(double _x, double _y) {
    x = _x;
    y = _y;
  }
  /// @brief Construction from another Vector (x, y)
  /// @param v Vector to copy data from
  public Vector(Vector v) {
    x = v.x;
    y = v.y;
  }
  /// @brief Construction from a Scanner
  /// @param s Scanner
  ///
  /// Format expected (x, y)
  public Vector(Scanner s) {
    char p1 = s.findInLine("[(]").charAt(0);
    if(p1 != '(')
      throw new InputMismatchException("Missing opening parenthesis");

    s.useDelimiter(",");
    x = s.nextDouble();

    char c = s.findInLine(",").charAt(0);
    if(c != ',')
      throw new InputMismatchException("Missing comma");

    s.useDelimiter("[\\p{javaWhitespace})]+");
    y = s.nextDouble();

    char p2 = s.findInLine("[)]").charAt(0);
    if(p2 !=  ')')
      throw new InputMismatchException("Missing closing parenthesis");
  }

  /// @return X coordinate
  public double x() {return x;}
  /// @return Y coordinate
  public double y() {return y;}
  /// @param _x new X coordinate
  public void x(double _x) {x = _x;}
  /// @param _y new Y coordinate
  public void y(double _y) {y = _y;}
  /// @return sqrt(x^2 + y^2)
  public double magnitude() {
    return Math.sqrt(x*x + y*y);
  }
  /// @param v Vector
  /// @return Dot product of this and v
  public double dot(Vector v) {
    return x*v.x + y*v.y;
  }

  public double cross(Vector v) {
    return x * v.y - y * v.x;
  }

  /// @brief Vector addition
  /// @param v Vector
  /// @return New vector (x + v.x, y + v.y)
  public Vector add(Vector v) {
    return new Vector(x + v.x, y + v.y);
  }
  /// @brief Vector addition
  /// @param v Vector
  ///
  /// (x, y) becomes (x + v.x, y + v.y)
  public void addeq(Vector v) {
    x += v.x;
    y += v.y;
  }

  /// @brief Vector subtraction
  /// @param v Vector
  /// @return New vector (x - v.x, y - v.y)
  public Vector sub(Vector v) {
    return new Vector(x - v.x, y - v.y);
  }
  /// @brief Vector addition
  /// @param v Vector
  ///
  /// (x, y) becomes (x + v.x, y + v.y)
  public void subeq(Vector v) {
    x -= v.x;
    y -= v.y;
  }

  /// @brief Vector scale
  /// @param d Scale
  /// @return New vector (d*x, d*y)
  public Vector multiply(double d) {
    return new Vector(x*d, y*d);
  }
  /// @brief Vector scale
  /// @param d Scale
  ///
  /// (x, y) becomes (x*d, y*d)
  public void multiplyeq(double d) {
    x *= d;
    y *= d;
  }

  /// @brief Vector normalize -- Convert to unit vector
  /// @return New vector of magnitude 1
  public Vector norm() {
    return multiply(1/magnitude());
  }
  /// @brief Vector normalize -- Convert to unit vector
  ///
  /// Alter vector to magnitude 1
  public void normeq() {
    this.multiplyeq(1/magnitude());
  }

  public static Vector getNorm(Vector p0, Vector p1) {
    Vector p = p1.sub(p0);
    Vector n = new Vector(p.y(), -p.x());
    n.normeq();
    return n;
  }

  /// @override
  /// @return String representation of vector
  public String toString() {
    return String.format("(%.4f, %.4f)", x, y);
  }

  /// @brief Unit test
  public static void main(String[] args) {
    //TODO
  }
}
