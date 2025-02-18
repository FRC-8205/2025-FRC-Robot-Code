package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;

public class Elevator {

    // devices
    private SparkMax leftMotorController;
    private SparkMax rightMotorController;
    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;

    public Elevator() {
        leftMotorController = new SparkMax(TunerConstants.getLeftElevatorMotorID(), SparkLowLevel.MotorType.kBrushless);
        rightMotorController = new SparkMax(TunerConstants.getRightElevatorMotorID(), SparkLowLevel.MotorType.kBrushless);

        leftMotorEncoder = leftMotorController.getEncoder();
        rightMotorEncoder = rightMotorController.getEncoder();

        updateElevatorWidget();
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

    public void moveElevatorUp() {
        leftMotorController.set(0.5); // Set motor speed to 50% of full speed
        rightMotorController.set(0.5); // Set motor speed to 50% of full speed
    }

    public InstantCommand createMoveCommand() {
        return new InstantCommand(() -> moveElevatorUp());
    }

    public void stopElevator() {
        leftMotorController.set(0); // Stop motor
        rightMotorController.set(0); // Stop motor
    }

    // create the widget in elastic
    private void updateElevatorWidget() {
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
