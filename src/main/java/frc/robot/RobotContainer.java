// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
//import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Winch;
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

    public final Vision vision = new Vision();

    public final Elevator elevator = new Elevator();

    //public final Coral coral = new Coral();

    public final Winch winch = new Winch();

    public final CustomKeyboard keyboard = new CustomKeyboard(1);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    //Testing for PathPlanner AutoChooser - Fullagar
    //private final SendableChooser<Command> buildAutChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
       
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
        // Elevator Down - this is correct
        // m_driverController.a().onTrue(elevator.moveElevatorUpCommand());
        // m_driverController.a().onFalse(elevator.stopElevatorCommand());

        // Testing following april tag
        // m_driverController.x().onTrue(() -> {
        //     double[] movement = vision.moveToClosestAprilTag();
            
        //     if (movement.length == 2) {
        //         drivetrain.applyRequest(() -> drive.withRotationalRate(movement[0]));
        //         drivetrain.applyRequest(() -> drive.withVelocityX(movement[1]));
        //     } else {
        //         System.out.println("Error with vision estimate");
        //     }
            
        // });

        //Testing Winch
        m_driverController.a().onTrue(winch.turnInCommand());
        m_driverController.a().onFalse(winch.stopMotorCommand());

        //Testing Winch Release
        m_driverController.b().onTrue(winch.turnOutCommand());
        m_driverController.b().onFalse(winch.stopMotorCommand());

        // Elevator Up - this is correct
        // m_driverController.b().onTrue(elevator.moveElevatorDownCommand());
        // m_driverController.b().onFalse(elevator.stopElevatorCommand());

        // Elevator to Level 1 
        m_driverController.x().onTrue(elevator.setElevatorCommand(10));

        // Elevator to Level 2
        m_driverController.y().onTrue(elevator.setElevatorCommand(40));
        
        // Reset Field-Centric Heading 
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // WINCH COMMANDS
        // turn winch in
        m_driverController.leftTrigger().onTrue(winch.turnInCommand());
        m_driverController.leftTrigger().onFalse(winch.stopMotorCommand());

        // turn winch out
        m_driverController.rightTrigger().onTrue(winch.turnOutCommand());
        m_driverController.rightTrigger().onFalse(winch.stopMotorCommand());

        
        // CUSTOM BINDINGS
        keyboard.moveLevel1().onTrue(elevator.setElevatorCommand(1));
        keyboard.moveLevel2().onTrue(elevator.setElevatorCommand(2));
        keyboard.moveLevel3().onTrue(elevator.setElevatorCommand(3));
        keyboard.moveLevel4().onTrue(elevator.setElevatorCommand(4));

    //     /* CORAL BUTTONS */
    //     // // Coral Output
    //     m_driverController.rightBumper().onTrue(coral.indexerCommand(0.1));
    //     m_driverController.rightBumper().onFalse(coral.stopIndexingCommand());

    //     // Coral Rotate Down
    //     // m_driverController.rightBumper().onTrue(coral.rotateCommand(0.1));
    //     // m_driverController.rightBumper().onFalse(coral.stopRotatingCommand());

    //     // Coral Rotate Up 
    //     m_driverController.leftBumper().onTrue(coral.rotateCommand(-0.1));
    //     m_driverController.leftBumper().onFalse(coral.stopRotatingCommand());

    //     // Coral Rotate Down PID
    //     m_driverController.rightTrigger().onTrue(coral.setCoralCommand(2));
        
    //     // Coral Rotate Up PID
    //     m_driverController.leftTrigger().onTrue(coral.setCoralCommand(-2));
        
    
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
