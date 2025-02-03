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

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;


import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * A simulated drivetrain that integrates a dynamics model of the swerve drive with your existing code.
 * 
 * This class extends your CommandSwerveDrivetrain and overrides updateSimState() to update the robot’s
 * pose using a simulation model based on a modified version of the SwerveDriveSim example.
 */
public class SimulatedDrivetrain extends CommandSwerveDrivetrain {
    // --- Simulation Model Fields ---
    private final SwerveDriveSimModel simModel;

    // For this example we capture the last voltage commands that were sent to the swerve modules.
    // In your code you might update these in your setControl() override.
    private double[] lastDriveVoltages;
    private double[] lastSteerVoltages;

    /**
     * Constructs the simulated drivetrain.
     *
     * @param drivetrainConstants Drivetrain-wide constants.
     * @param kinematics          The swerve drive kinematics. The order of modules here must match your constants.
     * @param modules             Module constants.
     */
    @SafeVarargs
    public SimulatedDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveDriveKinematics kinematics,
            SwerveModuleConstants<?, ?, ?>... modules) {
        // Call the parent constructor. You can choose whichever constructor fits your use.
        super(drivetrainConstants, modules);
        // Initialize the last voltage arrays based on number of modules.
        int numModules = modules.length;
        lastDriveVoltages = new double[numModules];
        lastSteerVoltages = new double[numModules];
        // Create the simulation model.
        // For this example, we use some nominal parameters.
        // You can later replace these with values from your TunerConstants or other configuration.
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 2.0); // (kS, kV)
        SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(0.2, 1.5);
        DCMotor driveMotor = DCMotor.getFalcon500(1); // example motor; adjust as needed
        DCMotor steerMotor = DCMotor.getFalcon500(1);
        double driveGearing = 8.14;
        double steerGearing = 21.43;
        double driveWheelRadius = 0.0508; // 2 inches in meters

        simModel = new SwerveDriveSimModel(
                driveFF, driveMotor, driveGearing, driveWheelRadius,
                steerFF, steerMotor, steerGearing,
                kinematics);
        // Initialize simulation pose to match current odometry.
        simModel.reset(getState().Pose, false);
    }

    /**
     * (Optional) Override your drivetrain control command to capture the voltage commands.
     * In your real code, when you set the control outputs, call recordVoltageCommands().
     *
     * @param driveVoltages  Array of drive motor voltage commands.
     * @param steerVoltages  Array of steer motor voltage commands.
     */
    public void recordVoltageCommands(double[] driveVoltages, double[] steerVoltages) {
        // Save the commands (clamped to battery voltage).
        double battVoltage = RobotController.getBatteryVoltage();
        for (int i = 0; i < driveVoltages.length; i++) {
            lastDriveVoltages[i] = MathUtil.clamp(driveVoltages[i], -battVoltage, battVoltage);
        }
        for (int i = 0; i < steerVoltages.length; i++) {
            lastSteerVoltages[i] = MathUtil.clamp(steerVoltages[i], -battVoltage, battVoltage);
        }
    }

    /**
     * This method is called periodically (by a Notifier in simulation) to update the simulation.
     *
     * @param dt             Time step in seconds.
     * @param batteryVoltage The current battery voltage.
     */
    @Override
    public void updateSimState(double dt, double batteryVoltage) {
        // Pass the last recorded voltage commands to the simulation model.
        // (If no new commands were recorded, these values remain from the previous cycle.)
        simModel.setDriveInputs(lastDriveVoltages);
        simModel.setSteerInputs(lastSteerVoltages);
        // Update the simulation model.
        simModel.update(dt);
        // Get the new pose from the simulation model.
        Pose2d simPose = simModel.getPose();
        // Feed the simulated pose back into the drivetrain odometry.
        resetPose(simPose);
        // Optionally, you could also update module positions/angles if your base class uses them.
    }

    // --- Accessor for simulation model, if needed for telemetry or debugging ---
    public SwerveModulePosition[] getSimulatedModulePositions() {
        return simModel.getModulePositions();
    }

    public Pose2d getSimulatedPose() {
        return simModel.getPose();
    }

    // --- Inner Simulation Model Class ---
    /**
     * This inner class is a modified version of your provided SwerveDriveSim.
     * It simulates the dynamics of the swerve drive based on drive and steer voltage inputs.
     */
    public static class SwerveDriveSimModel {
        private final LinearSystem<N2, N1, N2> drivePlant;
        private final double driveKs;
        private final DCMotor driveMotor;
        private final double driveGearing;
        private final double driveWheelRadius;
        private final LinearSystem<N2, N1, N2> steerPlant;
        private final double steerKs;
        private final DCMotor steerMotor;
        private final double steerGearing;

        private final SwerveDriveKinematics kinematics;
        private final int numModules;

        private final double[] driveInputs;
        private final List<Matrix<N2, N1>> driveStates;
        private final double[] steerInputs;
        private final List<Matrix<N2, N1>> steerStates;

        private final Random rand = new Random();

        // The "actual" robot pose in simulation (noiseless).
        private Pose2d pose = new Pose2d();
        private double omegaRadsPerSec = 0;

        /**
         * Constructs the simulation model using feedforward models and motor characteristics.
         */
        public SwerveDriveSimModel(
                SimpleMotorFeedforward driveFF,
                DCMotor driveMotor,
                double driveGearing,
                double driveWheelRadius,
                SimpleMotorFeedforward steerFF,
                DCMotor steerMotor,
                double steerGearing,
                SwerveDriveKinematics kinematics) {
            // Create drive linear system from feedforward constants.
            this.drivePlant = new LinearSystem<>(
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0,
                            0.0, -driveFF.getKv() / driveFF.getKa()),
                    VecBuilder.fill(0.0, 1.0 / driveFF.getKa()),
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0,
                            0.0, 1.0),
                    VecBuilder.fill(0.0, 0.0));
            this.driveKs = driveFF.getKs();
            this.driveMotor = driveMotor;
            this.driveGearing = driveGearing;
            this.driveWheelRadius = driveWheelRadius;

            // Create steer linear system from feedforward constants.
            this.steerPlant = new LinearSystem<>(
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0,
                            0.0, -steerFF.getKv() / steerFF.getKa()),
                    VecBuilder.fill(0.0, 1.0 / steerFF.getKa()),
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0,
                            0.0, 1.0),
                    VecBuilder.fill(0.0, 0.0));
            this.steerKs = steerFF.getKs();
            this.steerMotor = steerMotor;
            this.steerGearing = steerGearing;

            this.kinematics = kinematics;
            numModules = kinematics.toSwerveModuleStates(new ChassisSpeeds()).length;
            driveInputs = new double[numModules];
            driveStates = new ArrayList<>(numModules);
            steerInputs = new double[numModules];
            steerStates = new ArrayList<>(numModules);
            for (int i = 0; i < numModules; i++) {
                driveStates.add(VecBuilder.fill(0.0, 0.0));
                steerStates.add(VecBuilder.fill(0.0, 0.0));
            }
        }

        /**
         * Sets the drive motor input voltages.
         */
        public void setDriveInputs(double... inputs) {
            double battVoltage = RobotController.getBatteryVoltage();
            for (int i = 0; i < driveInputs.length; i++) {
                driveInputs[i] = MathUtil.clamp(inputs[i], -battVoltage, battVoltage);
            }
        }

        /**
         * Sets the steer motor input voltages.
         */
        public void setSteerInputs(double... inputs) {
            double battVoltage = RobotController.getBatteryVoltage();
            for (int i = 0; i < steerInputs.length; i++) {
                steerInputs[i] = MathUtil.clamp(inputs[i], -battVoltage, battVoltage);
            }
        }

        /**
         * Calculates the new state (position and velocity) using the discretized model and adds
         * static friction effects.
         */
        protected static Matrix<N2, N1> calculateX(
                Matrix<N2, N2> discA, Matrix<N2, N1> discB, Matrix<N2, N1> x, double input, double ks) {
            Matrix<N2, N1> Ax = discA.times(x);
            double nextStateVel = Ax.get(1, 0);
            double inputToStop = nextStateVel / -discB.get(1, 0);
            double ksSystemEffect = MathUtil.clamp(inputToStop, -ks, ks);
            nextStateVel += discB.get(1, 0) * ksSystemEffect;
            inputToStop = nextStateVel / -discB.get(1, 0);
            double signToStop = Math.signum(inputToStop);
            double inputSign = Math.signum(input);
            double ksInputEffect = 0;
            if (Math.abs(ksSystemEffect) < ks) {
                double absInput = Math.abs(input);
                ksInputEffect = -MathUtil.clamp(ks * inputSign, -absInput, absInput);
            } else if ((input * signToStop) > (inputToStop * signToStop)) {
                double absInput = Math.abs(input - inputToStop);
                ksInputEffect = -MathUtil.clamp(ks * inputSign, -absInput, absInput);
            }
            Matrix<N2, N1> Bu = discB.times(VecBuilder.fill(input + ksSystemEffect + ksInputEffect));
            return Ax.plus(Bu);
        }

        /**
         * Update the simulation model by one timestep.
         *
         * @param dtSeconds Timestep in seconds.
         */
        public void update(double dtSeconds) {
            var driveDiscAB = Discretization.discretizeAB(drivePlant.getA(), drivePlant.getB(), dtSeconds);
            var steerDiscAB = Discretization.discretizeAB(steerPlant.getA(), steerPlant.getB(), dtSeconds);
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[numModules];
            for (int i = 0; i < numModules; i++) {
                double prevDrivePos = driveStates.get(i).get(0, 0);
                driveStates.set(i,
                        calculateX(driveDiscAB.getFirst(), driveDiscAB.getSecond(),
                                driveStates.get(i), driveInputs[i], driveKs));
                double currDrivePos = driveStates.get(i).get(0, 0);
                steerStates.set(i,
                        calculateX(steerDiscAB.getFirst(), steerDiscAB.getSecond(),
                                steerStates.get(i), steerInputs[i], steerKs));
                double currSteerPos = steerStates.get(i).get(0, 0);
                moduleDeltas[i] = new SwerveModulePosition(currDrivePos - prevDrivePos, new Rotation2d(currSteerPos));
            }
            // Use the kinematics to compute the chassis twist and integrate to update the pose.
            var twist = kinematics.toTwist2d(moduleDeltas);
            pose = pose.exp(new Twist2d(twist.dx, twist.dy, twist.dtheta));
            omegaRadsPerSec = twist.dtheta / dtSeconds;
        }

        /**
         * Resets the simulation pose (and optionally resets module states).
         */
        public void reset(Pose2d pose, boolean preserveMotion) {
            this.pose = pose;
            if (!preserveMotion) {
                for (int i = 0; i < numModules; i++) {
                    driveStates.set(i, VecBuilder.fill(0.0, 0.0));
                    steerStates.set(i, VecBuilder.fill(0.0, 0.0));
                }
                omegaRadsPerSec = 0;
            }
        }

        /**
         * Returns the current simulated robot pose.
         */
        public Pose2d getPose() {
            return pose;
        }

        /**
         * Returns the module positions.
         */
        public SwerveModulePosition[] getModulePositions() {
            SwerveModulePosition[] positions = new SwerveModulePosition[numModules];
            for (int i = 0; i < numModules; i++) {
                positions[i] = new SwerveModulePosition(driveStates.get(i).get(0, 0),
                        new Rotation2d(steerStates.get(i).get(0, 0)));
            }
            return positions;
        }
    }
}
