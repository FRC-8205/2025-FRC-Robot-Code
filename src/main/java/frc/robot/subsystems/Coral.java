package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    private final ArmFeedforward armFeedforward;

    private double targetPos;
    private final double rotatePosOffset;

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

        // Set initial target position (in rotations)
        targetPos = 0.418;
        rotatePosOffset = 0.418;

        // Initialize PID controller (tune gains as necessary)
        pidControllerRotate = new PIDController(0.5, 0.0, 0.1);
        pidControllerRotate.setTolerance(0.02);
        pidControllerRotate.setSetpoint(targetPos);

        armFeedforward = new ArmFeedforward(.1, 1.0, .0);
    }

    private void launchCoral() {
        coralMotor1.set(0.5);
    }

    private void intakeAlgae() {
        algaeIndexMotor.set(0.6);
    }

    private void launchAlgae() {
        algaeIndexMotor.set(-0.75);
    }

    private double getRotateContinuousEncoderPosition() {
        double currentRawPosition = rotateEncoder.getPosition(); // 0 to 1 range

        // Check for wrap-around conditions
        if (currentRawPosition - lastEncoderPosition > 0.5) {
            rotationCount--;
        } else if (currentRawPosition - lastEncoderPosition < -0.5) {
            rotationCount++;
        }
    
        lastEncoderPosition = currentRawPosition;
        return rotationCount + currentRawPosition;
    }

    private void setLauncherRotation(double setPos) {
        targetPos = setPos;
        pidControllerRotate.setSetpoint(setPos);
    }

    private void moveLauncherToTarget(double currentPos) {
        double pidOutput = pidControllerRotate.calculate(currentPos);
        double angleRadians = (currentPos - rotatePosOffset) * 2.0 * Math.PI;
        double feedforwardOutput = armFeedforward.calculate(angleRadians, 0);
        double output = pidOutput + feedforwardOutput;
        
        rotateMotor.setVoltage(output);
        
        SmartDashboard.putNumber("Arm PID Output", pidOutput);
        SmartDashboard.putNumber("Arm Feedforward Output", feedforwardOutput);
        SmartDashboard.putNumber("Arm Motor Output", output);
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
        return new InstantCommand(() -> setLauncherRotation(setPos));
    }

    public InstantCommand stopCoralLaunchCommand() {
        return new InstantCommand(() -> stopCoralLaunch());
    }

    public InstantCommand stopAlgaeIndexCommand() {
        return new InstantCommand(() -> stopAlgaeIndex());
    }

    private void configureDashboard(double currentPos) {
        SmartDashboard.putNumber("Arm Rotate Encoder Position", currentPos);
    }

    @Override
    public void periodic() {
        // Cache the current encoder reading once per cycle.
        double continuousPosition = getRotateContinuousEncoderPosition();
        moveLauncherToTarget(continuousPosition);
        configureDashboard(continuousPosition);
    }
}
