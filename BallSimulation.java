import java.io.File;
import java.io.FileNotFoundException;
import java.lang.Exception;
import java.util.InputMismatchException;
import java.util.Scanner;

/// @brief This program simulates the motion of a bouncing ball in a vacuum.
public class BallSimulation {
  public static void main(String args[]) {
    if(args.length != 1) {
      System.out.println("Usage: java BallSimulation <file>");
      System.exit(1);
    }

    // Make a simulator on the parameters and simulate
    try {
      Scanner s = new Scanner(new File(args[0]));
      Simulator simulator = new Simulator(s);
      System.out.println("Finished reading data");
      simulator.simulate();
      System.exit(0);
    }
    catch(FileNotFoundException e) {
      System.out.println("Cannot find file " + args[0]);
      System.exit(1);
    }
    catch(InputMismatchException e) {
      System.out.println("Misformatted simulation file");
      System.out.println(e.getMessage());
      System.exit(1);
    }
    catch(Exception e) {
      System.out.println("Unknown error");
      System.out.println(e.getMessage());
      System.exit(1);
    }
  }
}
