// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private double matchTime;

  private static boolean redAlliance;

  PowerDistribution PDH;

  public Robot() {
    m_robotContainer = new RobotContainer();

    // try {
    //   Thread.sleep(5000);
    //   var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
    //   visionEst.ifPresent(est -> {
    //       var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();
    //       m_robotContainer.drivetrain.addVisionMeasurement(
    //           est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
    //       );
    //       m_robotContainer.drivetrain.resetPose(est.estimatedPose.toPose2d());
    //   });
    //   SmartDashboard.putBoolean("Correct Vision Pose", true);
    // } catch (InterruptedException e) {
    //   SmartDashboard.putBoolean("Correct Vision Pose", false);
    // }

    PDH = new PowerDistribution(1, ModuleType.kRev);

    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putData(PDH);
    PDH.setSwitchableChannel(true);

    redAlliance = checkRedAlliance();
  }

  @Override
  public void robotPeriodic() {
    if (DriverStation.isTeleop()) {
      Elastic.selectTab("Teleop");
    } else {
      Elastic.selectTab("Autonomous");
    }

    CommandScheduler.getInstance().run();

    // Correct pose estimate with vision measurements
    var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
        var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();
        m_robotContainer.drivetrain.addVisionMeasurement(
            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
        );
        SmartDashboard.putNumber("Vision x", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("Vision y", est.estimatedPose.toPose2d().getY());
    });


    matchTime = getMatchTime();  
    // Prevent showing -1 when disabled
    if (matchTime >= 0) {
        SmartDashboard.putNumber("Match Time", matchTime);
    } else {
        SmartDashboard.putNumber("Match Time", 0);
    }

    

    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    // m_robotContainer.elevator.updateDashboard();

  }

      /** Gets the current alliance, true is red */
    public static boolean getAlliance() {
        return redAlliance;
    }

    public static boolean checkRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        } else {
            DataLogManager.log("ERROR: Alliance not found. Defaulting to Blue");
            return false;
        }
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
      visionEst.ifPresent(est -> {
          var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();
          m_robotContainer.drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
          );
          m_robotContainer.drivetrain.resetPose(est.estimatedPose.toPose2d());
      });

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // resetPose(); // maybe try deleting this line of code to see if that fixes swerve.

    var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
      visionEst.ifPresent(est -> {
          var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();
          m_robotContainer.drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
          );
          m_robotContainer.drivetrain.resetPose(est.estimatedPose.toPose2d());
      });
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // Update drivetrain simulation
    m_robotContainer.drivetrain.simulationPeriodic();

    // Update camera simulation
    // m_robotContainer.vision.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    
    // var debugField = m_robotContainer.vision.getSimDebugField();
    // debugField.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);
    //debugField.getObject("EstimatedRobotModules").setPoses(m_robotContainer.drivetrain.getModulePoses());
    
    // Using max(0.1, voltage) here isn't a *physically correct* solution,
    // but it avoids problems with battery voltage measuring 0.
    RoboRioSim.setVInVoltage(Math.max(0.1, RobotController.getBatteryVoltage()));
  }

  public void resetPose() {
    // Example Only - startPose should be derived from some assumption
    // of where your robot was placed on the field.
    // The first pose in an autonomous path is often a good choice.
    var startPose = new Pose2d(1, 1, new Rotation2d());
    m_robotContainer.drivetrain.resetPose(startPose);
    m_robotContainer.vision.resetSimPose(startPose);
  }

  public double getMatchTime() {
    return DriverStation.getMatchTime(); // Returns time remaining in match
  }

  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage(); // Returns voltage (e.g., 12.5V)
  }


}
