package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;


public class Elevator extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;

    public Elevator() {
        leftMotor = new SparkMax(TunerConstants.getLeftElevatorMotorID(), SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(TunerConstants.getRightElevatorMotorID(), SparkMax.MotorType.kBrushless);

        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();

        configureDashboard();
    }

    private void moveElevatorUp() {
        // speed is a value between -1 and 1
        // motors need to spin in opposite directions
        leftMotor.set(-0.5);
        rightMotor.set(0.5);
    }

    private void moveElevatorDown() {
        // speed is a value between -1 and 1
        // motors need to spin in opposite directions
        leftMotor.set(0.5);
        rightMotor.set(-0.5);
    }

    private void stopElevator() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public InstantCommand moveElevatorUpCommand() {
        return new InstantCommand(() -> moveElevatorUp());
    }

    public InstantCommand moveElevatorDownCommand() {
        return new InstantCommand(() -> moveElevatorDown());
    }

    public InstantCommand stopElevatorCommand() {
        return new InstantCommand(() -> stopElevator());
    }

    private double getLeftMotorVelocity() {
        return leftMotorEncoder.getVelocity();
    }

    private double getRightMotorVelocity() {
        return rightMotorEncoder.getVelocity();
    }

    private double getElevatorHeight() {
        return leftMotorEncoder.getPosition() / TunerConstants.getElevatorGearRatio() * TunerConstants.getElevatorSproketCircumference();
    }

    private void configureDashboard() {
        SmartDashboard.putNumber("Elevator Left Motor Velocity", getLeftMotorVelocity());
        SmartDashboard.putNumber("Elevator Right Motor Velocity", getRightMotorVelocity());

        SmartDashboard.putNumber("Elevator Left Motor Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Motor Current", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    }

    public void updateDashboard() {
        configureDashboard();
    }
}
