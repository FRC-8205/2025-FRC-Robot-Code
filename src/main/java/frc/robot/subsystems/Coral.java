package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;

public class Coral extends SubsystemBase {
    private SparkMax rotateMotor;
    private SparkMax indexMotor;

    private RelativeEncoder rotateEncoder;
    private RelativeEncoder indexEncoder;

    private PIDController pidController;
    private boolean locked;

    private double currentPos;

    public Coral() {
        indexMotor = new SparkMax(TunerConstants.getCoralLauncherLaunchingMotorID(), SparkMax.MotorType.kBrushless);
        rotateMotor = new SparkMax(TunerConstants.getCoralLauncherRotatingMotorID(), SparkMax.MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        indexEncoder = indexMotor.getEncoder();

        locked = false;
        currentPos = 0;

        pidController = new PIDController(.05, 0.0, 0.01);
        pidController.setTolerance(0.02); 
    }

    private void indexer(double speed) {
        indexMotor.set(speed);
        locked = false;
    }

    private void stopRotating() {
        rotateMotor.set(0);
    }

    private void stopIndexing() {
        indexMotor.set(0);
    }

    private void rotate(double speed) {
        rotateMotor.set(speed);
        locked = false;
    }

    public InstantCommand indexerCommand(double speed) {
        return new InstantCommand(() -> indexer(speed));
    }

    public InstantCommand stopRotatingCommand() {
        return new InstantCommand(() -> stopRotating());
    }

    public InstantCommand rotateCommand(double speed) {
        return new InstantCommand(() -> rotate(speed));
    }  

    public InstantCommand stopIndexingCommand() {
        return new InstantCommand(() -> stopIndexing());
    }

    private void setElevatorPosition(double setPos) {
        locked = true;
        pidController.setSetpoint(currentPos + setPos);
        currentPos += setPos;
    }

    private void moveElevatorToTarget() {
        // Calculate the output using the PID controller
        double output = pidController.calculate(rotateEncoder.getPosition());
        output = Math.max(Math.min(output, 0.2), -0.2); // Limit the output to reduce top speed

        // Set the motor power
        rotateMotor.set(output);

        // If at setpoint, stop the motor
        if (pidController.atSetpoint()) {
            rotateMotor.stopMotor(); // Stop the motor when it reaches the target position
        }
    }

    public InstantCommand setCoralCommand(double setPos) {
        return new InstantCommand(() -> {
            setElevatorPosition(setPos);
            moveElevatorToTarget();
        });
    }

    @Override
    public void periodic() {
        if(locked) {
            moveElevatorToTarget(); 
        }
    }
}
