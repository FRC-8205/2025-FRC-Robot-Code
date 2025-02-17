package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.generated.TunerConstants;

public class Elevator {
    private SparkMax leftMotorController;
    private SparkMax rightMotorController;
    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;

    public Elevator(int leftMotorControllerID, int rightMotorControllerID) {
        leftMotorController = new SparkMax(leftMotorControllerID, SparkLowLevel.MotorType.kBrushless);
        rightMotorController = new SparkMax(rightMotorControllerID, SparkLowLevel.MotorType.kBrushless);

        leftMotorEncoder = leftMotorController.getEncoder();
        rightMotorEncoder = rightMotorController.getEncoder();

        configureElevatorWidget();
    }

    // motor is the side the motor is on, either left or right.
    public double getMotorPosition(String motor) {
        if (motor.equals("left")) {
            return leftMotorEncoder.getPosition();
        } else {
            return rightMotorEncoder.getPosition();
        }
    }

    // motor is the side the motor is on, either left or right.
    public double getMotorVelocity(String motor) {
        if (motor.equals("left")) {
            return leftMotorEncoder.getVelocity();
        } else {
            return rightMotorEncoder.getVelocity();
        }
    }

    // distance is the distance in inches from the lowest point, not current position
    public void moveElevatorFromBottom(double distance) {
        // TODO: figure out which side moves it up and which one moves it down
        // calculate number of rotations required
        double rotations = distance / (TunerConstants.getElevatorChainLength() * TunerConstants.getElevatorGearRatio() * TunerConstants.getElevatorRotations());

    }

    // create the widget in elastic
    private void configureElevatorWidget() {
        SmartDashboard.putData("Elevator", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Elevator");

                // Left Motor
                builder.addDoubleProperty("Left Motor Position",
                 () -> getMotorPosition("left"), null);
                builder.addDoubleProperty("Left Motor Velocity",
                 () -> getMotorVelocity("left"), null);

                // Right Motor
                builder.addDoubleProperty("Right Motor Position",
                 () -> getMotorPosition("right"), null);
                builder.addDoubleProperty("Right Motor Velocity",
                 () -> getMotorVelocity("right"), null);
            }
        });
    }
}
