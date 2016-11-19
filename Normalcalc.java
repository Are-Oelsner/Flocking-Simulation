/* Name: Are Oelsner
 * Program I created to help me calculate normals for my obstacles file
 * Contains a lot of failed attempts, but the first line it prints out
 * when given 2 points as input is the unit vector of one of the normals
 *
 * I realized that I didn't need to use angles only after creating my angle()
 * and norm() methods
*/
public class Normalcalc {
        public static double angle(Vector v) {
                return Math.atan(v.y()/v.x())*(180/Math.PI);
        }

        public static Vector norm(double angle) {
                double y = -Math.cos(angle);
                double x = Math.sin(angle);
                Vector n = new Vector(x, y);
                return n;
        }

        public static Vector getNorm(Vector p0, Vector p1) {
                Vector p = p1.sub(p0);
                Vector n = new Vector(p.y(), -p.x());
                return n.normeq();
        }

        public static void main(String[] args) {
                double x1 = Double.parseDouble(args[0]);
                double y1 = Double.parseDouble(args[1]);
                double x2 = Double.parseDouble(args[2]);
                double y2 = Double.parseDouble(args[3]);

                Vector p0 = new Vector(x1, y1);
                Vector p1 = new Vector(x2, y2);
                Vector p = p1.sub(p0);
                Vector n = new Vector(p.y(), -p.x());
                n.normeq();

                System.out.println("Normal Vector: " + n.toString());


                p0.normeq();
                p1.normeq();
                Vector v1 = p1.sub(p0);
                Vector v2 = v1.norm();


                System.out.println("Angle: " + String.valueOf(angle(p)));
                System.out.println("Normal: " + String.valueOf(angle(n)));
                System.out.println("Normal2: " + String.valueOf(180 + angle(n)));

                System.out.println("Vector p: " + norm(angle(p)).toString());
                System.out.println("Vector n: " + norm(angle(n)).toString());
                System.out.println("Vector n1: " + norm(angle(v1)).toString());
                System.out.println("Vector n: " + norm(angle(v2)).toString());
                System.out.println("Vector n1: " + v1.norm().toString());

                Vector n0 = p0.norm();
                Vector n1 = p1.norm();
                double slope = p.y()/p.x();
                double reciprocal = -p.x()/p.y();
                System.out.println("Slope: " + String.valueOf(slope));
                System.out.println("Normal: " + String.valueOf(reciprocal));
                System.out.println("Difference: " + p1.sub(p0).toString());
                System.out.println("Magnitude: " + p1.sub(p0).magnitude());
                System.out.println("Magtest: " + p.magnitude());
                double angle = Math.asin(p.y()/p.magnitude())*(180/Math.PI);
        }
}
