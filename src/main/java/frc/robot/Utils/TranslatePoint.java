package frc.robot.Utils;

public class TranslatePoint {
    // returns the positions for all 6 sides of the reef given the position of one side
    public static double[][] getMidpoints(double x, double y) {
        double[] midpointsX = new double[6];
        double[] midpointsY = new double[6];
        midpointsX[0] = x;
        midpointsY[0] = y;

        // apothem of the reef
        final double apothem = 0.83185;

        double newX, newY;
        for (int i = 0; i < 5; i++) {
            // temp values
            newX = 0;
            newY = 0;

            midpointsX[i + 1] = newX;
            midpointsY[i + 1] = newY; 
        }
        
        double[][] midpoints = new double[2][6];
        midpoints[0] = midpointsX;
        midpoints[1] = midpointsY;
        return midpoints;
    }
}
