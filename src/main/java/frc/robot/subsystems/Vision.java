package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Utils.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera aprilTagCameraFront;
    //private final PhotonCamera aprilTagCameraBack;
    private final PhotonCamera driverCam;
    private final PhotonPoseEstimator photonEstimatorFront;
    private final PhotonPoseEstimator photonEstimatorBack;

    private Matrix<N3, N1> curStdDevs;

    private final PathConstraints pathConstraints = new PathConstraints(RobotContainer.getMaxSpeed(), 1, RobotContainer.getMaxAngularRate(), 1);

    // Simulation
    private PhotonCameraSim cameraSim1;
    private PhotonCameraSim cameraSim2;
    private PhotonCameraSim cameraSim3;
    private VisionSystemSim visionSim;

    public Vision() {
        aprilTagCameraFront = new PhotonCamera(kCameraNameFront);
        // aprilTagCameraBack = new PhotonCamera(kCameraNameBack);
        driverCam = new PhotonCamera(kCameraNameDriver);

        TunerConstants.Vision.kTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

 

        photonEstimatorFront =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam1);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorBack =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam2);
        photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorFront.setLastPose(
                Robot.getAlliance()
                        ? TunerConstants.Vision.startingPoseRed
                        : TunerConstants.Vision.startingPoseBlue);
        photonEstimatorBack.setLastPose(
                Robot.getAlliance()
                        ? TunerConstants.Vision.startingPoseRed
                        : TunerConstants.Vision.startingPoseBlue);
        
                        

                        

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create PhotonCameraSim instances which will update the linked PhotonCamera's values with visible targets.
            cameraSim1 = new PhotonCameraSim(aprilTagCameraFront, cameraProp);
            // cameraSim2 = new PhotonCameraSim(aprilTagCameraBack, cameraProp);
            cameraSim3 = new PhotonCameraSim(driverCam, cameraProp);
            // Add the simulated cameras to view the targets on this simulated field.
            visionSim.addCamera(cameraSim1, kRobotToCam1);
            visionSim.addCamera(cameraSim2, kRobotToCam2);
            visionSim.addCamera(cameraSim3, kRobotToCam3);

            cameraSim1.enableDrawWireframe(true);
            cameraSim2.enableDrawWireframe(true);
            cameraSim3.enableDrawWireframe(true);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : aprilTagCameraFront.getAllUnreadResults()) {
            visionEst = photonEstimatorFront.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }
        // for (var change : aprilTagCameraBack.getAllUnreadResults()) {
        //     visionEst = photonEstimatorBack.update(change);
        //     updateEstimationStdDevs(visionEst, change.getTargets());

        //     if (Robot.isSimulation()) {
        //         visionEst.ifPresentOrElse(
        //                 est ->
        //                         getSimDebugField()
        //                                 .getObject("VisionEstimation")
        //                                 .setPose(est.estimatedPose.toPose2d()),
        //                 () -> {
        //                     getSimDebugField().getObject("VisionEstimation").setPoses();
        //                 });
        //     }
        // }
        return visionEst;
    }

    

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

}
