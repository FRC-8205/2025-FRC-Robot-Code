package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.hardware.ParentDevice;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * A simulated drivetrain that integrates a dynamics model of the swerve drive with your existing code.a
 * 
 * This class extends your CommandSwerveDrivetrain and overrides updateSimState() to update the robot’s
 * pose using a simulation model based on a modified version of the SwerveDriveSim example.
 */
public class SimulatedDrivetrain extends CommandSwerveDrivetrain {
    // define the CANcoders
    private CoreCANcoder m_frontLeftCoreCANcoder = new CoreCANcoder(TunerConstants.getFrontLeftEncoder());
    private CoreCANcoder m_frontRightCoreCANcoder = new CoreCANcoder(TunerConstants.getFrontRightEncoder());
    private CoreCANcoder m_backLeftCoreCANcoder = new CoreCANcoder(TunerConstants.getBackLeftEncoder());
    private CoreCANcoder m_backRightCoreCANcoder = new CoreCANcoder(TunerConstants.getBackRightEncoder());

    // define CANcoder simulation states
    private CANcoderSimState m_frontLeftCoreCANcoderSim = new CANcoderSimState(m_frontLeftCoreCANcoder);
    private CANcoderSimState m_frontRightCoreCANcoderSim = new CANcoderSimState(m_frontRightCoreCANcoder);
    private CANcoderSimState m_backLeftCoreCANcoderSim = new CANcoderSimState(m_backLeftCoreCANcoder);
    private CANcoderSimState m_backRightCoreCANcoderSim = new CANcoderSimState(m_backRightCoreCANcoder);

    // define gyroscope
    private CorePigeon2 gyroscope = new CorePigeon2(TunerConstants.getPigeonID());

    // define gyroscope simulation state
    private Pigeon2SimState gyroscopeSim = new Pigeon2SimState(gyroscope);

    // define wheel locations
    Translation2d[] m_wheelPositions = {
        new Translation2d(TunerConstants.getBackLeftXPos(), TunerConstants.getBackLeftYPos()),
        new Translation2d(TunerConstants.getBackRightXPos(), TunerConstants.getBackRightYPos()),
        new Translation2d(TunerConstants.getFrontLeftXPos(), TunerConstants.getFrontLeftYPos()),
        new Translation2d(TunerConstants.getFrontRightXPos(), TunerConstants.getFrontRightYPos())
    };

    // define the swerve drivetrain
    private SimSwerveDrivetrain m_simDrivetrain = new SimSwerveDrivetrain(m_wheelPositions, gyroscopeSim, TunerConstants.BackLeft, TunerConstants.BackRight, TunerConstants.FrontLeft, TunerConstants.FrontRight);

    // define each of the motors and encoders in the drivetrain
    private String CANbusName = TunerConstants.getCANbusName();
    private DeviceConstructor<CommonTalon> m_driveMotorConstructor = (id, name) -> {
        return new CoreTalonFX(id, name);
    };
    private DeviceConstructor<CommonTalon> m_steerMotorConstructor = (id, name) -> {
        return new CoreTalonFX(id, name);
    };
    private DeviceConstructor<ParentDevice> m_encoderConstructor = (id, name) -> {
        return new CoreCANcoder(id);
    };

    // define each module
    private SwerveModule<CommonTalon, CommonTalon, ParentDevice> m_frontLeftModule = new SwerveModule(m_driveMotorConstructor, m_steerMotorConstructor, m_encoderConstructor, TunerConstants.FrontLeft, 0, 0);
    @Override
    public void simulationPeriodic() {
        // m_simDrivetrain.update(0.02, TunerConstants.getBatteryVoltage(), )
    }
}
