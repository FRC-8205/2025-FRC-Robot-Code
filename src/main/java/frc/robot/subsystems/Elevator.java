package frc.robot.subsystems;

//import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.generated.TunerConstants;


public class Elevator extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private AbsoluteEncoder leftMotorEncoder;
    
    private PIDController pidController;
    private boolean locked;

    private double targetPosition;


    private double lastEncoderPosition;
    private int rotationCount;


    public Elevator() {
        leftMotor = new SparkMax(TunerConstants.getLeftElevatorMotorID(), SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(TunerConstants.getRightElevatorMotorID(), SparkMax.MotorType.kBrushless);

        leftMotorEncoder = leftMotor.getAbsoluteEncoder();

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

    private double getElevatorContinuousEncoderPosition() {
        double currentRawPosition = leftMotorEncoder.getPosition(); // Get absolute encoder value (0 to 1)
        
        // Check for wrap-around conditions
        if (currentRawPosition - lastEncoderPosition > 0.5) {
            // Encoder jumped backward (e.g., from 0.99 to 0.01), decrease rotation count
            rotationCount--;
        } else if (currentRawPosition - lastEncoderPosition < -0.5) {
            // Encoder jumped forward (e.g., from 0.01 to 0.99), increase rotation count
            rotationCount++;
        }
    
        // Update last position
        lastEncoderPosition = currentRawPosition;
    
        // Compute continuous position
        return rotationCount + currentRawPosition;
    }    

    // Move the elevator towards the target position using the PID controller
    private void moveElevatorToTarget() {
        // Calculate the output using the PID controller
        double output = pidController.calculate(getElevatorContinuousEncoderPosition());
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

    private double getElevatorBottomHeight() {
        return getElevatorContinuousEncoderPosition() / TunerConstants.getElevatorGearRatio() * TunerConstants.getElevatorSproketCircumference();
    }

    private void configureDashboard() {
        SmartDashboard.putNumber("Elevator Motor Velocity", getLeftMotorVelocity());

        SmartDashboard.putNumber("Elevator Left Motor Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Motor Current", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Elevator Left Motor Bus Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator Right Motor Bus Voltage", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Elevator Encoder Position", getElevatorContinuousEncoderPosition());

        SmartDashboard.putNumber("Elevator Height", getElevatorBottomHeight());

        SmartDashboard.putNumber("Target Position", targetPosition);

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
