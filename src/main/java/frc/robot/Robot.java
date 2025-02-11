// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Correct pose estimate with vision measurements
    var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
      est -> {
        // Change our trust in the measurement based on the tags we can see
        var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();
    
        m_robotContainer.drivetrain.addVisionMeasurement(
        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
    resetPose();
  }

  @Override
  public void teleopPeriodic() {}

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
    m_robotContainer.vision.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    
    var debugField = m_robotContainer.vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);
    debugField.getObject("EstimatedRobotModules").setPoses(m_robotContainer.drivetrain.getModulePoses());
    
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
}
