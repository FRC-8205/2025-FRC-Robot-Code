package frc.robot.Utils;

public class TranslatePoint {
    // returns the positions for all 12 coral branches
    // all calculations done in inches and then converted to meters to prevent rounding errors.
    public static double[][] getMidpoints() {
        double[] midpointsX = new double[12];
        double[] midpointsY = new double[12];

        // pre-computed constants to save time
        final double root3 = Math.sqrt(3);
        final double inchesToMeters = 127 / 5000;

        // apothem of the reef in inches
        final double reefApothem = 32.75;
        final double reefEdgeLength = 144;
        final double fieldWidth = 317.25;
        final double reefEdgeWidth = (fieldWidth - (2 * reefApothem * root3)) / 2;
        final double robotDiameter = 36;
        // center to center between the two pipes
        final double branchDistance = 13;
        final double branchDiameter = 1.25;

        // angle of rotation from the starting point. Starting point is perpendicular to the driver station
        double angle = 0;

        // first positions for left branch of the side of the reef facing the driver station
        double newX = reefEdgeLength - (double) (robotDiameter / 2);
        double newY = reefEdgeWidth + (reefApothem * root3) - (branchDistance / 2);
        
        // x and y coordinates shifted with the center of the hexagon at (0, 0)
        double shiftedX, shiftedY;

        for (int i = 0; i < 12; i++) {
            midpointsX[i] = newX;
            midpointsY[i] = newY;

            // not changing sides, just doing the right branch instead
            if (i % 2 == 0) {
                newX += branchDistance * Math.sin(angle);
                newY += branchDistance * Math.cos(angle);
            } else {
                // move back over to the left branch
                newX -= branchDistance * Math.sin(angle);
                newY -= branchDistance * Math.cos(angle);

                // shift points down so that the center of the reef is at the origin
                shiftedX = newX - reefApothem - reefEdgeLength;
                shiftedY = newY - reefEdgeWidth - (reefApothem * root3);

                // rotate the points 60 degrees counter-clockwise (sine and cosine values subbed in)
                shiftedX = (shiftedX / 2) - (shiftedY * root3 / 2);
                shiftedY = (shiftedX * root3 / 2) + (shiftedY / 2);

                // shift the points back to the correct locations
                newX = shiftedX + reefApothem + reefEdgeLength;
                newY = shiftedY + reefEdgeWidth + (reefApothem * root3);

                angle += 60;
            }
        }
        
        // convert all the x and y values to meters
        for int (i = 0; i < 12; i++) {
            midpointsX[i] *= inchesToMeters;
            midpointsY[i] *= inchesToMeters;
        }
        
        double[][] midpoints = new double[2][12];
        midpoints[0] = midpointsX;
        midpoints[1] = midpointsY;
        return midpoints;
    }
}
