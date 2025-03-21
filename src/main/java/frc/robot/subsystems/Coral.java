package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
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

    private AbsoluteEncoder rotateEncoder;
    private RelativeEncoder algaeIndexEncoder;
    private RelativeEncoder coralEncoder;

    private PIDController pidControllerRotate;

    private double currentPos;
    private double targetPos;

    private double lastEncoderPosition = 0.0;
    private int rotationCount = 0;
    

    public Coral() {
        // initialize motors
        rotateMotor = new SparkMax(TunerConstants.getCoralRotateMotorID(), SparkMax.MotorType.kBrushless);
        algaeIndexMotor = new SparkMax(TunerConstants.getAlgaeIndexMotorID(), SparkMax.MotorType.kBrushless);
        coralMotor1 = new SparkMax(TunerConstants.getCoralLanchMotor1ID(), SparkMax.MotorType.kBrushless);
        coralMotor2 = new SparkMax(TunerConstants.getCoralLaunchMotor2ID(), SparkMax.MotorType.kBrushless);

        // initialize encoders
        rotateEncoder = rotateMotor.getAbsoluteEncoder();
        algaeIndexEncoder = algaeIndexMotor.getEncoder();
        coralEncoder = coralMotor1.getEncoder();

        // set follower for coral launcher
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(coralMotor1, true);

        SparkMaxConfig rotateConfig = new SparkMaxConfig();
        rotateConfig.idleMode(IdleMode.kBrake);
    

        rotateMotor.configure(rotateConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        coralMotor2.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
;
        currentPos = 0;
        targetPos = 0.418;

        pidControllerRotate = new PIDController(.5, 0.0, 0.00);
        pidControllerRotate.setTolerance(0.02); 
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
        algaeIndexMotor.set(0.6);
    }

    private void launchAlgae() {
        algaeIndexMotor.set(-0.75);
    }

    private double getRotateContinuousEncoderPosition() {
        double currentRawPosition = rotateEncoder.getPosition(); // Get absolute encoder value (0 to 1)
        
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

    private void setLauncherRotation(double setPos) {
        targetPos = setPos;
        pidControllerRotate.setSetpoint(setPos);
    }

    private void moveLauncherToTarget() {
        double output = pidControllerRotate.calculate(getRotateContinuousEncoderPosition());
        //output = Math.max(Math.min(output, 0.1), -0.1);

        rotateMotor.set(output);

        if (pidControllerRotate.atSetpoint()) {
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

    private void configureDashboard() {
        SmartDashboard.putNumber("Arm Rotate Encoder Position", getRotateContinuousEncoderPosition());
    }


    @Override
    public void periodic() {
            moveLauncherToTarget();

        configureDashboard();
    }
}
