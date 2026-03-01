// Vision.java
package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public final class VisionConstants {
    public static final String kLeftCamName = "LeftCam";
    public static final String kRightCamName = "RightCam";

    public static final Transform3d kRobotToLeftCam = new Transform3d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5), Units.inchesToMeters(7.25), new Rotation3d(0, Units.degreesToRadians(45),Units.degreesToRadians(45)));
    public static final Transform3d kRobotToRightCam = new Transform3d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5), Units.inchesToMeters(7.25), new Rotation3d(0, Units.degreesToRadians(45),Units.degreesToRadians(-45)));

    // Filtering thresholds
    public static final double kMaxAmbiguity = 0.20;
    public static final double kMaxTagDistanceMetersMulti = 5.0;
    public static final double kMaxTagDistanceMetersSingle = 3.0;

    // Std devs: (x meters, y meters, heading radians)
    // Smaller = more trust.
    public static final Matrix<N3, N1> kStdDevsMultiTag = VecBuilder.fill(0.30, 0.30, Math.toRadians(90000));

    public static final Matrix<N3, N1> kStdDevsSingleTag = VecBuilder.fill(1.50, 1.50, Math.toRadians(90000));
  }

  private final PhotonCamera leftCamera = new PhotonCamera(VisionConstants.kLeftCamName);
  private final PhotonCamera rightCamera = new PhotonCamera(VisionConstants.kRightCamName);

  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;

  public record VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
  }

  public Vision() {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // placeholder name

    leftEstimator = new PhotonPoseEstimator(
        layout, VisionConstants.kRobotToLeftCam);

    rightEstimator = new PhotonPoseEstimator(
        layout, VisionConstants.kRobotToRightCam);
  }

  public Optional<VisionMeasurement> getLeftMeasurement(Pose2d referencePose) {
    return getMeasurement(leftCamera, leftEstimator, referencePose);
  }

  public Optional<VisionMeasurement> getRightMeasurement(Pose2d referencePose) {
    return getMeasurement(rightCamera, rightEstimator, referencePose);
  }

  private Optional<VisionMeasurement> getMeasurement(
      PhotonCamera camera,
      PhotonPoseEstimator estimator,
      Pose2d referencePose) {

    // 1) Read latest pipeline result so we can inspect tag count / ambiguity /
    // distance
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets())
      return Optional.empty();

    int tagCount = result.getTargets().size();

    // 2) Ambiguity + distance checks (typically from "best target")
    var best = result.getBestTarget();

    // NOTE: method names vary slightly by PV version; your IDE will guide you.
    double ambiguity = best.getPoseAmbiguity(); // common
    double distanceMeters = best.getBestCameraToTarget().getTranslation().getNorm(); // common

    if (ambiguity > VisionConstants.kMaxAmbiguity)
      return Optional.empty();

    // Be stricter when it's only one tag
    if (tagCount >= 2) {
      if (distanceMeters > VisionConstants.kMaxTagDistanceMetersMulti)
        return Optional.empty();
    } else { // tagCount == 1
      if (distanceMeters > VisionConstants.kMaxTagDistanceMetersSingle)
        return Optional.empty();
    }

    // 3) Compute estimated pose (latency-compensated timestamp comes from
    // EstimatedRobotPose)
    estimator.setReferencePose(referencePose);
    Optional<EstimatedRobotPose> estOpt = estimator.estimateCoprocMultiTagPose(result);
    if (estOpt.isEmpty())
      return Optional.empty();

    EstimatedRobotPose est = estOpt.get();

    // 4) Choose confidence profile
    Matrix<N3, N1> stdDevs = (tagCount >= 2) ? VisionConstants.kStdDevsMultiTag : VisionConstants.kStdDevsSingleTag;

    return Optional.of(new VisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
  }
}