package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;

public class Funnel extends SubsystemBase {
    private SparkMax rotateMotor;
    private RelativeEncoder rotateEncoder;

    private PIDController pidController;

    private boolean locked;

    private double currentPos;
    private double targetPos;

    public Funnel() {
        rotateMotor = new SparkMax(TunerConstants.getFunnelMotorID(), SparkMax.MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();

        pidController = new PIDController(.05, 0.0, 0.01);

        locked = false;
        currentPos = 0;
        targetPos = 0;
    }

    private void rotateDown() {
        locked = false;
        rotateMotor.set(-0.1);
    }

    private void rotateUp() {
        locked = false;
        rotateMotor.set(0.1);
    }

    private void stopRotating() {
        locked = true;
        targetPos = rotateEncoder.getPosition();
        pidController.setSetpoint(targetPos);
        rotateMotor.set(0);
    }

    private void setFunnelRotation(double setPos) {
        locked = true;
        targetPos = setPos;
        pidController.setSetpoint(targetPos);
    }

    private void moveFunnelToTarget() {
        double output = pidController.calculate(rotateEncoder.getPosition());
        output = Math.max(Math.min(output, 0.2), -0.2);

        rotateMotor.set(output);

        if (pidController.atSetpoint()) {
            rotateMotor.set(0);
        }
    }

    public InstantCommand rotateDownCommand() {
        return new InstantCommand(() -> rotateDown());
    }

    public InstantCommand rotateUpCommand() {
        return new InstantCommand(() -> rotateUp());
    }

    public InstantCommand stopRotatingCommand() {
        return new InstantCommand(() -> stopRotating());
    }

    public InstantCommand setFunnelRotationCommand(double setPos) {
        return new InstantCommand(() -> {
            setFunnelRotation(setPos);
            moveFunnelToTarget();
        });
    }

    @Override
    public void periodic() {
        if (locked) {
            moveFunnelToTarget();
        }
    }
}
