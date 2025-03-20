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
    private SparkMax algaeIndexMotor;
    private SparkMax coralMotor1;
    private SparkMax coralMotor2;

    private RelativeEncoder rotateEncoder;
    private RelativeEncoder algaeIndexEncoder;
    private RelativeEncoder coralEncoder;

    private PIDController pidController;
    private boolean locked;

    private double currentPos;
    private double targetPos;

    public Coral() {
        // initialize motors
        rotateMotor = new SparkMax(TunerConstants.getCoralRotateMotorID(), SparkMax.MotorType.kBrushless);
        algaeIndexMotor = new SparkMax(TunerConstants.getAlgaeIndexMotorID(), SparkMax.MotorType.kBrushless);
        coralMotor1 = new SparkMax(TunerConstants.getCoralLanchMotor1ID(), SparkMax.MotorType.kBrushless);
        coralMotor2 = new SparkMax(TunerConstants.getCoralLaunchMotor2ID(), SparkMax.MotorType.kBrushless);

        // initialize encoders
        rotateEncoder = rotateMotor.getEncoder();
        algaeIndexEncoder = algaeIndexMotor.getEncoder();
        coralEncoder = coralMotor1.getEncoder();

        // set follower for coral launcher
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(coralMotor1, true);

        coralMotor2.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        locked = false;
        currentPos = 0;
        targetPos = 0;

        pidController = new PIDController(.05, 0.0, 0.01);
        pidController.setTolerance(0.02); 
    }

    private void launchCoral() {
        coralMotor1.set(0.5);
        // try {
        //     Thread.sleep(1000);
        //     coralMotor1.set(0);
        // } catch (InterruptedException e) {
        //     System.out.println("Wait command for coral launch failed.");
        //     coralMotor1.set(0);
        // }
    }

    private void intakeAlgae() {
        algaeIndexMotor.set(-0.5);
    }

    private void launchAlgae() {
        algaeIndexMotor.set(0.5);
    }

    private void setLauncherRotation(double setPos) {
        locked = true;
        targetPos = setPos;
        pidController.setSetpoint(setPos);
    }

    private void moveLauncherToTarget() {
        double output = pidController.calculate(rotateEncoder.getPosition());
        output = Math.max(Math.min(output, 0.2), -0.2);

        rotateMotor.set(output);

        if (pidController.atSetpoint()) {
            rotateMotor.set(0);
        }
    }

    private void stopCoralLaunch() {
        coralMotor1.set(0);
    }

    private void stopAlgaeIndex() {
        algaeIndexMotor.set(0);
    }

    public InstantCommand launchCoralCommand() {
        return new InstantCommand(() -> launchCoral());
    }

    public InstantCommand intakeAlgaeCommand() {
        return new InstantCommand(() -> intakeAlgae());
    }

    public InstantCommand launchAlgaeCommand() {
        return new InstantCommand(() -> launchAlgae());
    }

    public InstantCommand setLauncherRotationCommand(double setPos) {
        return new InstantCommand(() -> {
            setLauncherRotation(setPos);
            moveLauncherToTarget();
        });
    }

    public InstantCommand stopCoralLaunchCommand() {
        return new InstantCommand(() -> stopCoralLaunch());
    }

    public InstantCommand stopAlgaeIndexCommand() {
        return new InstantCommand(() -> stopAlgaeIndex());
    }

    @Override
    public void periodic() {
        if (locked) {
            moveLauncherToTarget();
        }
    }
}
