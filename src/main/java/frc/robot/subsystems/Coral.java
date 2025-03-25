package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;

public class Coral extends SubsystemBase {
    private SparkMax rotateMotor;
    private SparkMax algaeIndexMotor;
    private SparkMax coralMotor1;
    private SparkMax coralMotor2;

    private RelativeEncoder rotateEncoder;
    private AbsoluteEncoder rotateAbsoluteEncoder;
    private RelativeEncoder algaeIndexEncoder;
    private RelativeEncoder coralEncoder;

    private PIDController pidControllerRotate;
    private PIDController pidAlgaeIntake;

    private ColorSensorV3 colorSensor;
    private boolean coralLoaded = false;
    private int moveCount = 0;

    //private final ArmFeedforward armFeedforward;

    private double targetPos;
    private double targetAlgae;
    private boolean locked;
    private boolean algaeLocked;
    private final double rotatePosOffset;

    private double lastEncoderPosition = 0.0;
    private int rotationCount = 0;

    public Coral() {
        // level 1 elevator - 6.142879486083984
        // level 2 angle - -5.499996185302734 elevator - 16.071365356445312
        // -.142 default angle
        // initialize motors
        rotateMotor = new SparkMax(TunerConstants.getCoralRotateMotorID(), SparkMax.MotorType.kBrushless);
        algaeIndexMotor = new SparkMax(TunerConstants.getAlgaeIndexMotorID(), SparkMax.MotorType.kBrushless);
        coralMotor1 = new SparkMax(TunerConstants.getCoralLanchMotor1ID(), SparkMax.MotorType.kBrushless);
        coralMotor2 = new SparkMax(TunerConstants.getCoralLaunchMotor2ID(), SparkMax.MotorType.kBrushless);

        // initialize encoders
        rotateEncoder = rotateMotor.getEncoder();
        rotateAbsoluteEncoder = rotateMotor.getAbsoluteEncoder();
        rotateEncoder.setPosition(rotateAbsoluteEncoder.getPosition());
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

        locked = true;
        algaeLocked = false;

        // Initialize PID controller (tune gains as necessary)
        pidControllerRotate = new PIDController(0.05, 0.0, 0.0);
        pidControllerRotate.setTolerance(0.02);
        pidControllerRotate.setSetpoint(targetPos);

        pidAlgaeIntake = new PIDController(0, 0, 0);
        pidAlgaeIntake.setTolerance(0.02);

        //armFeedforward = new ArmFeedforward(.1, 1.0, .0);
        // initialize color sensor
        final I2C.Port i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
        colorSensor.configureColorSensor(ColorSensorV3.ColorSensorResolution.kColorSensorRes20bit, ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms, ColorSensorV3.GainFactor.kGain3x);
    }

    private void rotateDownManual() {
        locked = false;
        rotateMotor.set(.3);
    }

    private void loseCoral() {
        locked = false;
        coralMotor1.set(-0.1);
    }

    public InstantCommand loseCoralCommand() {
        return new InstantCommand(() -> loseCoral());
    }
    private void rotateUpManual() {
        locked = false;
        rotateMotor.set(-.3);
    }

    private void stopArm() {
        rotateMotor.set(0);
    }

    private void launchCoral() {
        coralMotor1.set(0.15);
    }

    private void intakeAlgae() {
        algaeIndexMotor.set(0.6);
        algaeLocked = false;
    }

    private void launchAlgae() {
        algaeIndexMotor.set(-0.75);
    }

    // private double getRotateContinuousEncoderPosition() {
    //     double currentRawPosition = rotateEncoder.getPosition(); // 0 to 1 range

    //     // Check for wrap-around conditions
    //     if (currentRawPosition - lastEncoderPosition > 0.5) {
    //         rotationCount--;
    //     } else if (currentRawPosition - lastEncoderPosition < -0.5) {
    //         rotationCount++;
    //     }
    
    //     lastEncoderPosition = currentRawPosition;
    //     return rotationCount + currentRawPosition;
    // }

    private void setLauncherRotation(double setPos) {
        locked = true;
        targetPos = setPos;
        pidControllerRotate.setSetpoint(setPos);
    }

    private void moveLauncherToTarget(double currentPos) {
        double pidOutput = pidControllerRotate.calculate(currentPos);
        // double angleRadians = (currentPos - rotatePosOffset) * 2.0 * Math.PI;
        // double feedforwardOutput = armFeedforward.calculate(angleRadians, 0);
        double output = pidOutput;
        
        rotateMotor.set(output);
        
        SmartDashboard.putNumber("Arm PID Output", pidOutput);
        //SmartDashboard.putNumber("Arm Feedforward Output", feedforwardOutput);
        SmartDashboard.putNumber("Arm Motor Output", output);
    }

    private void moveAlgaeLauncherToTarget(double currentPos) {
        double pidOutput = pidAlgaeIntake.calculate(currentPos);
        algaeIndexMotor.set(pidOutput);
    }

    public InstantCommand moveAlgaeLauncherToTargetCommand(double currentPos) {
        return new InstantCommand(() -> moveAlgaeLauncherToTarget(currentPos));
    }

    private void stopCoralLaunch() {
        coralMotor1.set(0);
    }

    private void stopAlgaeIndex() {
        algaeIndexMotor.set(0);
        algaeLocked = true;
        targetAlgae = algaeIndexEncoder.getPosition();
        moveAlgaeLauncherToTarget(targetAlgae);
    }

    public InstantCommand armUpCommand() {
        return new InstantCommand(() -> rotateUpManual());
    }

    public InstantCommand armDownCommand() {
        return new InstantCommand(() -> rotateDownManual());
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

    public InstantCommand stopArmCommand() {
        return new InstantCommand(() -> stopArm());
    }

    private void configureDashboard(double currentPos, ColorSensorV3.RawColor color) {
        SmartDashboard.putNumber("Arm Rotate Encoder Position", currentPos);
        SmartDashboard.putNumber("Color Sensor Color", (color.red + color.blue + color.green));
    }

    

    @Override
    public void periodic() {
        // Cache the current encoder reading once per cycle.
        double currentPosition = rotateEncoder.getPosition();
        if (locked) {
            moveLauncherToTarget(currentPosition);
        }

        if (algaeLocked) {}

        // color sensor
        ColorSensorV3.RawColor currentColor = colorSensor.getRawColor();
        configureDashboard(currentPosition, currentColor);
    }
}
