// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Utils.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Coral;
// import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.CustomKeyboard;


public class RobotContainer {
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static double getMaxSpeed() {
        return MaxSpeed;
    }

    public static double getMaxAngularRate() {
        return MaxAngularRate;
    }

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Vision vision = new Vision(drivetrain);

    public final Elevator elevator = new Elevator();

    public final Coral coral = new Coral();

    // public final Winch winch = new Winch();

    public final CustomKeyboard keyboard = new CustomKeyboard(1);

    public final Funnel funnel = new Funnel();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    //Testing for PathPlanner AutoChooser - Fullagar
    //private final SendableChooser<Command> buildAutChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
       
        // Register Commands
        NamedCommands.registerCommand("Elevator Level 3", elevator.setElevatorCommand(16.373));
        NamedCommands.registerCommand("Launch Coral", coral.launchCoralCommand());
        NamedCommands.registerCommand("Stop Launch Coral", coral.stopCoralLaunchCommand());

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        // ));

        m_driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* DRIVER BUTTONS */

        // Set elevator to bottom position
        m_driverController.a().onTrue(elevator.setElevatorCommand(0));

        // Launch Coral
        m_driverController.b().onTrue(coral.launchCoralCommand());
        m_driverController.b().onFalse(coral.stopCoralLaunchCommand());

        // Intake Algae - swap
        m_driverController.x().onTrue(coral.intakeAlgaeCommand());
        m_driverController.x().onFalse(coral.stopAlgaeIndexCommand());

        // Launch Algae
        m_driverController.y().onTrue(coral.launchAlgaeCommand());
        m_driverController.y().onFalse(coral.stopAlgaeIndexCommand());
        
        // Reset Field-Centric Heading 
        // m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Move to April Tag 10
        // m_driverController.rightBumper().onTrue(AutoBuilder.followPath(vision.moveToClosestAprilTag(10)));

        // rotate coral arm out
        m_driverController.leftBumper().onTrue(coral.setLauncherRotationCommand(.2));

        // rotate coral arm to vertical
        m_driverController.rightBumper().onTrue(coral.setLauncherRotationCommand(.418));

        // Rotate funnel up
        m_driverController.rightTrigger().onTrue(funnel.rotateUpCommand());
        m_driverController.rightTrigger().onFalse(funnel.stopRotatingCommand());

        // Rotate funnel down
        m_driverController.leftTrigger().onTrue(funnel.rotateDownCommand());
        m_driverController.leftTrigger().onFalse(funnel.stopRotatingCommand());
        
        // CUSTOM BINDINGS
        keyboard.moveLevel1().onTrue(elevator.setElevatorCommand(1));
        keyboard.moveLevel2().onTrue(elevator.setElevatorCommand(2));
        keyboard.moveLevel3().onTrue(elevator.setElevatorCommand(3));
        keyboard.moveLevel4().onTrue(elevator.setElevatorCommand(4));    
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
