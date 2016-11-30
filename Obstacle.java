//Name: Are Oelsner
//My obstacles class

import java.util.Scanner;
import java.awt.*;
import java.io.*;
import java.util.*;
import java.awt.geom.Path2D.Double;

public class Obstacle {
  private int numPoints;
  private Vector[] points;
  private Vector[] normals;
  private Vector[] edges;

  Obstacle() {}

  public Obstacle(String inFileName) throws FileNotFoundException {
    Scanner s = new Scanner(new File(inFileName));
    numPoints = s.nextInt();
    for(int i = 0; i < numPoints; i++) {
      double x = s.nextDouble();
      double y = s.nextDouble();
      points[i] = new Vector(x, y);
    }
    for(int i = 0; i < numPoints; i++) {
      double x = s.nextDouble();
      double y = s.nextDouble();
      normals[i] = new Vector(x, y);
    }
    for(int i = 0; i < numPoints; i++) {
      double x = s.nextDouble();
      double y = s.nextDouble();
      edges[i] = new Vector(x, y);
    }
  }

  public Obstacle(Scanner s) {
    boolean done = false;
    while(s.hasNextLine() && !done) {
      String line = s.nextLine();
      Scanner l = new Scanner(line);
      String tag = l.next();
      switch(tag) {
        case "Points":
          System.out.println("Reading points");
          for(int i = 0; i < numPoints; i++) {
            double x = l.nextDouble();
            double y = l.nextDouble();
            points[i] = new Vector(x, y);
          }
          break;
        case "NumPoints":
          System.out.println("Reading numPoints");
          numPoints = l.nextInt();
          points = new Vector[numPoints];
          normals = new Vector[numPoints];
          edges = new Vector[numPoints];
          break;
        case "Normals":
          System.out.println("Reading normals");
          for(int i = 0; i < numPoints; i++) {
            double x = l.nextDouble();
            double y = l.nextDouble();
            normals[i] = new Vector(x, y);
          }
          break;
        case "Edges":
          System.out.println("Reading edges");
          for(int i = 0; i < numPoints; i++) {
            double x = l.nextDouble();
            double y = l.nextDouble();
            edges[i] = new Vector(x, y);
          }
          break;
        case "~Obstacle":
          System.out.println("Reading ~Obstacle");
          done = true;
          break;
        default:
          throw new InputMismatchException("Unknown tag " + tag);
      }
      l.close();
    }
  }

  public static Obstacle[] readObstacles(Scanner s, int N) throws FileNotFoundException {
    Obstacle[] Obstacle = new Obstacle[N];
    for(int i = 0; i < N; i++) {
      Obstacle[i] = new Obstacle(s);
    }
    return Obstacle;
  }

  public void draw() {
    double[] xPoints = new double[numPoints];
    double[] yPoints = new double[numPoints];
    for(int i = 0; i < numPoints; i++) {
      xPoints[i] = points[i].x();
      yPoints[i] = points[i].y();
    }
    GUI.drawPolygon(xPoints, yPoints, numPoints);
  }


  public int getNumPoints() {return numPoints;}
  public Vector[] getPoints() {return points;}
  public Vector[] getNormals() {return normals;}
  public Vector[] edges() {return edges;}

  //returns Vector coordinates of an individual edge as a Vector array
  public Vector[] getEdge(int i) {
    Vector[] edge = new Vector[2];
    edge[0] = points[(int)(edges[i].x())];
    edge[1] = points[(int)(edges[i].y())];
    return edge;
  }
  public Vector getPoint(int i) {return points[i];}

  public String toString() {return String.valueOf(numPoints);}

}








