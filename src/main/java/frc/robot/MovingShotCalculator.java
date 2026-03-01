package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Computes turret heading + distance for hood/RPM lookup, including
 * lead compensation for shooting while translating.
 *
 * Assumptions:
 * - robotPose is FIELD-relative (Pose2d on the field)
 * - robotSpeeds is ROBOT-relative (ChassisSpeeds from swerve)
 * - targetField is a fixed FIELD point (speaker aim point)
 */
public class MovingShotCalculator {

  /** Simple interface so you can back it with a LUT/table/polynomial. */
  @FunctionalInterface
  public interface FlightTimeModel {
    /** @return estimated time-of-flight (seconds) for a shot at this distance (meters). */
    double flightTimeSeconds(double distanceMeters);
  }

  public record Solution(
      Rotation2d turretSetpoint,     // robot-relative turret angle (what your turret wants)
      double distanceMeters,          // straight-line distance robot->target now
      double effectiveDistanceMeters, // distance corrected for radial motion (optional but handy)
      double flightTimeSeconds,
      boolean valid
  ) {}

  private final FlightTimeModel flightTimeModel;

  // Safety clamps so your LUT doesn’t get garbage inputs
  private final double minDistanceMeters;
  private final double maxDistanceMeters;

  public MovingShotCalculator(
      FlightTimeModel flightTimeModel,
      double minDistanceMeters,
      double maxDistanceMeters
  ) {
    this.flightTimeModel = flightTimeModel;
    this.minDistanceMeters = minDistanceMeters;
    this.maxDistanceMeters = maxDistanceMeters;
  }

  /**
   * @param robotPose Field-relative estimated robot pose
   * @param robotRelSpeeds Robot-relative chassis speeds (m/s, rad/s)
   * @param targetField Field-relative target point (meters)
   */
  public Solution solve(Pose2d robotPose, ChassisSpeeds robotRelSpeeds, Translation2d targetField) {

    // Convert robot-relative speeds to field-relative speeds (needed for lead)
    ChassisSpeeds fieldRel =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelSpeeds, robotPose.getRotation());

    Translation2d vField = new Translation2d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond);

    // Vector from robot->target (NOW)
    Translation2d r = targetField.minus(robotPose.getTranslation());
    double dist = r.getNorm();
    if (dist < 1e-6) {
      return new Solution(new Rotation2d(), 0.0, 0.0, 0.0, false);
    }

    // Estimate flight time from current distance
    double t = Math.max(0.0, flightTimeModel.flightTimeSeconds(dist));

    // Predict robot translation during flight (LEAD)
    Translation2d robotFuture = robotPose.getTranslation().plus(vField.times(t));
    Translation2d rFuture = targetField.minus(robotFuture);

    // Aim at "future" vector (field bearing), then convert to robot-relative turret angle
    double bearingFutureFieldRad = Math.atan2(rFuture.getY(), rFuture.getX());
    double turretRad =
        MathUtil.angleModulus(bearingFutureFieldRad - robotPose.getRotation().getRadians());

    // Optional: radial distance correction (helps when driving toward/away)
    Translation2d unitToTarget = r.div(dist);
    double vRadial = vField.getX() * unitToTarget.getX() + vField.getY() * unitToTarget.getY();
    double distEff = dist - vRadial * t;

    // Clamp effective distance for LUT safety
    distEff = MathUtil.clamp(distEff, minDistanceMeters, maxDistanceMeters);

    return new Solution(
        Rotation2d.fromRadians(turretRad),
        dist,
        distEff,
        t,
        true
    );
  }
}