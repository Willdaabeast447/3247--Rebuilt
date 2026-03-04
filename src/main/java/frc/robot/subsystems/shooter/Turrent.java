// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.io.Console;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turrent extends SubsystemBase {

  private class Constants {
    private final static int MOTOR_ID = 40;
    private final static int CURRENT_LIMIT = 40;
    private final static double safePosition = 0;
    public static double MIN_ANGLE = -85;
    public static double MAX_ANGLE = 175;
    public static double SWITCH_ANGLE = 93.43;
    public static double DEG_PER_REV = 6.54;
    public static double VISION_TURNING_KP = 0.0125;
  }

  private final SparkMax motor = new SparkMax(Constants.MOTOR_ID, SparkMax.MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = motor.getEncoder();// taken from old code
  private DoubleEntry turrentNT;
  private final DigitalInput turretHomeSwitch = new DigitalInput(0);
  private BooleanEntry halleffectNT;
  private SparkClosedLoopController turrretClosedLoopController;
  private Boolean halleffect_prevState = false;
  private double visionOutput = 0;
  private BooleanEntry canMoveNT;
  private DoubleEntry yawNT;
  private double trim = 0;
  private boolean trim_held=false;
  private DoubleEntry trimNt;

  public Turrent() {
    configureMotors();
    turrretClosedLoopController = motor.getClosedLoopController();

    encoder.setPosition(13.76);// set position to 90 degrees for starting pos

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Turrent");
    turrentNT = table.getDoubleTopic("position").getEntry(0);
    turrentNT.set(0);
    halleffectNT = table.getBooleanTopic("HallEffect").getEntry(false);
    canMoveNT = table.getBooleanTopic("canMove").getEntry(false);
    yawNT = table.getDoubleTopic("yaw").getEntry(0);
    trimNt=table.getDoubleTopic("Turret Trim").getEntry(0);
    trimNt.set(trim);
  }

  private void configureMotors() {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not
    // powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or
    // momentum.
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.CURRENT_LIMIT)
        .inverted(true); // Protects motor hardware by limiting current.
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.05)
        .i(0)
        .d(0)
        .outputRange(-1, 1)

        .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0).feedForward
        .kS(0.4);
    config.softLimit
        .forwardSoftLimit(degreesToRotations(Constants.MAX_ANGLE))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(degreesToRotations(Constants.MIN_ANGLE))
        .reverseSoftLimitEnabled(true);

    // Apply the configurations to the motor controllers.
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double clampTargetAngle(double angle) {
    return MathUtil.clamp(angle, Constants.MIN_ANGLE, Constants.MAX_ANGLE);
  }

  private void setOnSwitch() {
    if (!turretHomeSwitch.get() && !halleffect_prevState) {
      encoder.setPosition(degreesToRotations(Constants.SWITCH_ANGLE));
    }
    halleffect_prevState = !turretHomeSwitch.get();
  }

  private double rotationsToDegree(double rotations) {
    return rotations * Constants.DEG_PER_REV;
  }

  private double degreesToRotations(double degrees) {
    return degrees / Constants.DEG_PER_REV;
  }

  private void setMotorAngle(double angle) {
    turrretClosedLoopController.setSetpoint(
        degreesToRotations(MathUtil.clamp(angle, Constants.MIN_ANGLE, Constants.MAX_ANGLE)), ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  private void setMotorPower(double power) {
    motor.set(MathUtil.clamp(power, -1, 1));
  }

  private Rotation2d angleToTarget(Pose2d robot, Pose2d target) {
    return target.getTranslation()
        .minus(robot.getTranslation())
        .getAngle();
  }

  private void stop() {
    setMotorPower(0);
  }

  private void trimShot(boolean right,boolean left)
  {

          if ((right||left) && !trim_held)
          {
            if (right) {
              trim=5;              
            }
            else {
              trim=5;
            }
          }
          trim_held=(right||left);
  }

  /*
   * public Command manualCommand()
   * {
   * return new FunctionalCommand(
   * () -> {
   * }, // No initialization needed
   * () -> setPower(turrentNT), // Set intake motor power dynamically
   * interrupted -> stop(), // Stop motor if interrupted
   * () -> false, // Runs indefinitely
   * this // Pass the subsystem
   * );
   * }
   * }
   */
  public Command AimAtYaw(DoubleSupplier yaw, BooleanSupplier canMove,BooleanSupplier trimRightSupplier,BooleanSupplier trimLeftSupplier) {

    return new FunctionalCommand(
        () -> {
          // pick a hight any hight
        },

        () -> {
          trimShot(trimRightSupplier.getAsBoolean(),trimLeftSupplier.getAsBoolean());
     
          if (canMove.getAsBoolean()) {
            
            visionOutput = 0;
          } else {
            visionOutput = Constants.VISION_TURNING_KP * -(yaw.getAsDouble()+trim);
          }
          setMotorPower(visionOutput);
          canMoveNT.set(canMove.getAsBoolean());
          yawNT.set(yaw.getAsDouble());
        },

        interrupted -> {

          stop();

        },

        () -> false,
        this);
  }

  public Command AimAtAngle(DoubleSupplier Angle) { // speed means RPM
    return new FunctionalCommand(
        () -> {
          // pick a hight any hight
        },

        () -> {
          setMotorAngle(Angle.getAsDouble());

        },

        interrupted -> {

        },

        () -> false,
        this);
  }

  public Command Stow() { // speed means RPM
    return new FunctionalCommand(
        () -> {
          // pick a hight any hight
        },

        () -> {
          setMotorAngle(90);

        },

        interrupted -> {

        },

        () -> turrretClosedLoopController.isAtSetpoint(),
        this);
  }

  public Command AimAtPose(Supplier<Pose2d> robotPose, Pose2d targetPose) { // speed means RPM
    return new FunctionalCommand(
        () -> {
          // pick a hight any hight
        },

        () -> {
          setMotorAngle(angleToTarget(robotPose.get(), targetPose).getDegrees());

        },

        interrupted -> {

        },

        () -> false,
        this);
  }

  @Override
  public void periodic() {
    halleffectNT.set(!turretHomeSwitch.get());
    turrentNT.set(encoder.getPosition());
    setOnSwitch();
    trimNt.set(trim);

  }
}
