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


// public class Elevator extends SubsystemBase {

//     // devices
//     private SparkMax leftMotor; // Leader
//     private SparkMax rightMotor; // Follower

//     private final SparkMax.ResetMode resetMode;
//     private final SparkMax.PersistMode persisteMode;

//     private RelativeEncoder leftMotorEncoder;


//     // PID
//     private final SparkMax.ControlType posControl = SparkMax.ControlType.kPosition;
//     private final SparkClosedLoopController lockPosition = leftMotor.getClosedLoopController();
//     private final PIDController elevatorPID = new PIDController(3, 1, 0);

//     // vars
//     private double revolutionCount;
//     private double setHeight;

//     public boolean locked = false;


//     public Elevator() {
//         leftMotor = new SparkMax(TunerConstants.getLeftElevatorMotorID(), SparkMax.MotorType.kBrushless);
//         rightMotor = new SparkMax(TunerConstants.getRightElevatorMotorID(), SparkMax.MotorType.kBrushless);

//         resetMode = SparkMax.ResetMode.kResetSafeParameters;
//         persisteMode = SparkMax.PersistMode.kPersistParameters;

//         // encoder and PID configs
//         leftMotorEncoder = leftMotor.getEncoder();
//         elevatorPID.setTolerance(0.02);

//         // motor configs
//         SparkMaxConfig elevatorConfig = new SparkMaxConfig();
//         elevatorConfig.idleMode(IdleMode.kBrake);
//         elevatorConfig.inverted(true);
        
//         SparkMaxConfig followerConfig = new SparkMaxConfig();
//         followerConfig.follow(leftMotor, true);


//         leftMotor.configure(elevatorConfig, resetMode, persisteMode);
//         rightMotor.configure(elevatorConfig, resetMode, persisteMode);
//         rightMotor.configure(followerConfig, resetMode, persisteMode);
//         rightMotor.configure(followerConfig, resetMode, persisteMode);

        
//         SmartDashboard.putData("Elevator PID", elevatorPID);

//         leftMotorEncoder.setPosition(0);

//     }


//     public double getMotorVelocity() {
//             return leftMotorEncoder.getVelocity();
//     }

//     public double getHeightEncoder() {
//         return (revolutionCount / TunerConstants.getElevatorGearRatio()) * TunerConstants.getElevatorSproketCircumference();
//     }


//     public void setElevatorSpeedManual(double value) {
//         leftMotor.set(value);
//     }

//     public double getEncoderValue() {
//         return revolutionCount;
//     }

//     public void setPosition(double value) {
//         locked = true;
//         lockPosition.setReference(value, posControl);
//     }

//     public void stopHere() {
//         locked = true;
//         lockPosition.setReference(revolutionCount, posControl);
//     }

//     public void setHeight(double height) {
//         locked = true;
//         setHeight = height;
//         double output = elevatorPID.calculate(getHeightEncoder(), height);
//         leftMotor.set(output);
//     }

//     public double getHeight() {
//         return getHeightEncoder();
//     }

//     public boolean atHeight() {
//         return elevatorPID.atSetpoint();
//     }

//     public void resetElevatorEncoder() {
//         leftMotorEncoder.setPosition(0);
//     }

//     public double getSetpoint() {
//         return setHeight;
//     }

//     public double getPosition() {
//         return leftMotorEncoder.getPosition();
//     }


//     @Override
//     public void periodic() {
//         revolutionCount = leftMotorEncoder.getPosition();

//         SmartDashboard.putNumber("Elevator Motor Velocity", getMotorVelocity());
//         SmartDashboard.putNumber("Encoder Height", getHeightEncoder());
//         SmartDashboard.putNumber("Elevator Left Supply Current", leftMotor.getOutputCurrent());
//         SmartDashboard.putNumber("Elevator Right Supply Current", rightMotor.getOutputCurrent());
//     }

// }

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
        leftMotor.set(0.5);
        rightMotor.set(-0.5);
    }

    private void moveElevatorDown() {
        // speed is a value between -1 and 1
        // motors need to spin in opposite directions
        leftMotor.set(-0.5);
        rightMotor.set(0.5);
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
