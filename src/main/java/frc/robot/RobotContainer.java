// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.controls.CoastOut;
// import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.util.ChangeVariableCommand;
import frc.robot.subsystems.Coral;


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
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Vision vision = new Vision();

    public final Elevator elevator = new Elevator();

    public final Coral coral = new Coral();

    private final double swerveSpeed = 1;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
       
        // Register Commands
        // InstantCommand elevatorLevel3 = elevator.setElevatorCommand(24.1385);
        // InstantCommand launchCoral = coral.launchCoralCommand();
        for (int i = 1; i < 13; i++) {
            NamedCommands.registerCommand("Robot to Coral Side " + i, drivetrain.pathToCoralandScore(i));
        }

        NamedCommands.registerCommand("Elevator Up", elevator.setElevatorCommand(6.32766469574765));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * swerveSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * swerveSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * swerveSpeed) // Drive counterclockwise with negative X (left)
            )
        );

        // // Start PhotonVision thread
        // photonThread.setName("PhotonVision");
        // photonThread.setDaemon(true);
        // photonThread.start();

        // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        // ));

        // m_driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // m_driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        m_driverController.a().onTrue(new InstantCommand(
          ()->drivetrain.resetRotation(new Rotation2d(0))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* DRIVER BUTTONS */
        /*
         * Needed Buttons:
         * Moving to elevator to all 4z levels and bottom (5 buttons) done
         * Launching and intake of algae (2 buttons) done
         * Intake and outtake of coral (2 buttons) done
         * Rotating launcher to vertical and 35 degrees (2 buttons) done
         * Button to make swerve drive slower (1 button)
         * level 1 - 17.28863246905321
         * level 2 - 6.32766469574765
         * level 3 - 14.5789935357442
         * level 4 - 
         * 
         */
        // Moving elevator to levels 1, 2, 3, and 4
        m_driverController.povUp().onTrue(elevator.setElevatorCommand(6.142879486083984));
        m_driverController.povRight().onTrue(elevator.setElevatorCommand(16.071365356445312));
        m_driverController.povDown().onTrue(elevator.setElevatorCommand(24.1385));
        m_driverController.povLeft().onTrue(elevator.setElevatorCommand(24.754)); // incorrect height

        // // Moving elevator to bottom
        m_driverController.leftTrigger().onTrue(elevator.setElevatorCommand(0));

        // // Launching coral
         m_driverController.x().onTrue(coral.launchCoralCommand());
         m_driverController.x().onFalse(coral.stopCoralLaunchCommand());

        // // Moving coral back
        // m_driverController.a().onTrue(coral.loseCoralCommand());
        // m_driverController.a().onFalse(coral.stopCoralLaunchCommand());

        // // Launching algae
        // m_driverController.x().onTrue(coral.launchAlgaeCommand());
        // m_driverController.x().onFalse(coral.stopAlgaeIndexCommand());
        //m_driverController.x().onTrue(drivetrain.pathToCoralandScore(9));
        //m_driverController.y().onTrue(AutoBuilder.pathfindToPose(new Pose2d(6.0, 6.0, new Rotation2d(0)), TunerConstants.Auto.PathfindingConstraints));
        // // Intake algae
        // m_driverController.y().onTrue(coral.intakeAlgaeCommand());
        // m_driverController.y().onFalse(coral.stopAlgaeIndexCommand());

        // // Setting arm to vertical
        // m_driverController.leftBumper().onTrue(coral.setLauncherRotationCommand(-0.3));
        m_driverController.leftBumper().onTrue(elevator.moveElevatorUpCommand());
        m_driverController.leftBumper().onFalse(elevator.stopElevatorCommand());

        // // Setting arm to rotation for levels 1, 2, and 3
        // set default arm position
        m_driverController.b().onTrue(coral.setLauncherRotationCommand(-7.428585529327393));
        m_driverController.rightBumper().onTrue(elevator.moveElevatorDownCommand());
        m_driverController.rightBumper().onFalse(elevator.stopElevatorCommand());
        m_driverController.y().onTrue(coral.rotateManualCommand());

        // Making swerve drive slower
        ChangeVariableCommand<Double> changeCommand = new ChangeVariableCommand<Double>((Double) swerveSpeed, new Double[] {1.0, 0.5});
        m_driverController.rightTrigger().onTrue(changeCommand);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
