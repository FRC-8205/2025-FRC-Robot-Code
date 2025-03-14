package frc.robot.subsystems;

//import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.generated.TunerConstants;


public class Elevator extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;
    
    private PIDController pidController;
    private boolean locked;

    private double targetPosition;

    private DutyCycleEncoder throughBoreEncoder;


    public Elevator() {
        leftMotor = new SparkMax(TunerConstants.getLeftElevatorMotorID(), SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(TunerConstants.getRightElevatorMotorID(), SparkMax.MotorType.kBrushless);

        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();

        throughBoreEncoder = new DutyCycleEncoder(0, 360, 0);

        locked = false;

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftMotor, true);

        rightMotor.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        pidController = new PIDController(.3, 0.0, 0.0);
        pidController.setTolerance(0.02); 
        // pidController.setIntegratorRange(-0.5, 0.5); 

        configureDashboard();
    }

    private void moveElevatorUp() {
        // speed is a value between -1 and 1
        locked = false;
        leftMotor.set(-0.1);
    }

    private void moveElevatorDown() {
        // speed is a value between -1 and 1
        locked = false;
        leftMotor.set(0.1);
    }

    // Set the target position for the elevator
    private void setElevatorPosition(double setPos) {
        locked = true;
        targetPosition = setPos;
        pidController.setSetpoint(setPos);
    }

    // Move the elevator towards the target position using the PID controller
    private void moveElevatorToTarget() {
        // Calculate the output using the PID controller
        double output = pidController.calculate(leftMotorEncoder.getPosition());
        output = Math.max(Math.min(output, 0.2), -0.2); // Limit the output to reduce top speed

        // Set the motor power
        leftMotor.set(output);

        // If at setpoint, stop the motor
        if (pidController.atSetpoint()) {
            leftMotor.stopMotor(); // Stop the motor when it reaches the target position
        }
    }

    private void stopElevator() {
        leftMotor.set(0);
    }

    // Command to set the elevator to a position
    public InstantCommand setElevatorCommand(double setPos) {
        return new InstantCommand(() -> {
            setElevatorPosition(setPos);
            moveElevatorToTarget();
        });
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

    private double getElevatorBottomHeight() {
        return leftMotorEncoder.getPosition() / TunerConstants.getElevatorGearRatio() * TunerConstants.getElevatorSproketCircumference();
    }

    private void configureDashboard() {
        SmartDashboard.putNumber("Elevator Left Motor Velocity", getLeftMotorVelocity());
        SmartDashboard.putNumber("Elevator Right Motor Velocity", getRightMotorVelocity());

        SmartDashboard.putNumber("Elevator Left Motor Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Motor Current", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Elevator Left Motor Bus Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator Right Motor Bus Voltage", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Encoder Position", leftMotorEncoder.getPosition());

        SmartDashboard.putNumber("Elevator Height", getElevatorBottomHeight());

        SmartDashboard.putNumber("Target Position", targetPosition);

        SmartDashboard.putNumber("Through Bore Encoder", throughBoreEncoder.get());

    }

    public void updateDashboard() {
        configureDashboard();
    }

    // Periodic method for subsystem updates
    @Override
    public void periodic() {
        if(locked) {
            moveElevatorToTarget(); 
        }
        updateDashboard(); // Update the dashboard with the latest information
    }
}
