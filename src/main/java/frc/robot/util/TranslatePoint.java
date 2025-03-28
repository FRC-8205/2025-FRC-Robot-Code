package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TranslatePoint {
    // returns the positions for all 12 coral branches
    // all calculations done in inches and then converted to meters to prevent rounding errors.
    public static Pose2d[] getPoses() {
        double[] midpointsX = new double[12];
        double[] midpointsY = new double[12];
        Rotation2d[] rotations = new Rotation2d[12];

        Pose2d[] poses = new Pose2d[12];

        // pre-computed constants to save time
        final double root3 = Math.sqrt(3);
        final double inchesToMeters = 127.0 / 5000;

        // apothem of the reef in inches
        final double reefApothem = 32.75;
        final double reefEdgeLength = 144;
        final double fieldWidth = 317.25;
        final double reefEdgeWidth = (fieldWidth - (2 * reefApothem * root3)) / 2;
        final double robotDiameter = 36;
        // center to center between the two pipes
        final double branchDistance = 13;

        // angle of rotation from the starting point. Starting point is perpendicular to the driver station
        double angle = 0;

        // first positions for left branch of the side of the reef facing the driver station
        double newX = reefEdgeLength - (double) (robotDiameter / 2);
        double newY = reefEdgeWidth + (reefApothem * root3) + (branchDistance / 2);
        
        // x and y coordinates shifted with the center of the hexagon at (0, 0)
        double shiftedX, shiftedY;

        for (int i = 0; i < 12; i++) {
            midpointsX[i] = newX;
            midpointsY[i] = newY;
            rotations[i] = new Rotation2d(Units.degreesToRadians(angle));

            // not changing sides, just doing the right branch instead
            if (i % 2 == 0) {
                newX -= branchDistance * Math.sin(Units.degreesToRadians(angle));
                newY -= branchDistance * Math.cos(Units.degreesToRadians(angle));
            } else {
                // move back over to the left branch
                newX += branchDistance * Math.sin(Units.degreesToRadians(angle));
                newY += branchDistance * Math.cos(Units.degreesToRadians(angle));

                // shift points down so that the center of the reef is at the origin
                shiftedX = newX - reefApothem - reefEdgeLength;
                shiftedY = newY - reefEdgeWidth - (reefApothem * root3);

                // rotate the points 60 degrees counter-clockwise (sine and cosine values subbed in)
                double tempX = shiftedX;
                double tempY = shiftedY;
                shiftedX = (tempX / 2) - (tempY * root3 / 2);
                shiftedY = (tempX * root3 / 2) + (tempY / 2);

                // shift the points back to the correct locations
                newX = shiftedX + reefApothem + reefEdgeLength;
                newY = shiftedY + reefEdgeWidth + (reefApothem * root3);

                angle += 60;
            }
        }
        
        // convert all the x and y values to meters
        for (int i = 0; i < 12; i++) {
            midpointsX[i] *= inchesToMeters;
            midpointsY[i] *= inchesToMeters;
        }

        // convert the x, y and angle values to Pose2d objects
        for (int i = 0; i < 12; i++) {
            poses[i] = new Pose2d(midpointsX[i], midpointsY[i], rotations[i]);
        }

        for (int i = 0; i < 12; i++) {
            SmartDashboard.putNumber("Pose " + i + " X", poses[i].getX());
            SmartDashboard.putNumber("Pose " + i + " Y", poses[i].getY());
        }
        
        return poses;
    }

    public static Pose2d flipPose (Pose2d pose) {
        Pose2d flippedPose;
        double x;
        double y;
        double angle;
        final double fieldLength = 690.875958;
        final double fieldWidth = 317.25;

        // flip the x and y coordinates
        x = pose.getX() + ((fieldLength/2 - pose.getX()) * 2);
        y = pose.getX() + ((fieldWidth/2 - pose.getY()) * 2);

        // flip the angle
        angle = pose.getRotation().getDegrees() + 180;

        flippedPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));

        return flippedPose;
    }
}
