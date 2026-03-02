// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private class Constants {
    private final static int MOTOR_ID = 41;
    private final static int MOTOR_TWO_ID = 42;
    private final static int MOTOR_KICKER_ID = 44;
    private final static int MOTOR_PREKICKER_ID = 45;
    private final static int SERVO_HUB_ID = 43;
    // Spark Flex current limit
    private final static int CURRENT_LIMIT = 80;
    public static int MIN_FLYWHEELSPEED = 4000;
    public static Pose2d hubPose = new Pose2d(4, 4, new Rotation2d());

    public static double SHOOTER_KP = 0.0002;
    public static double SHOOTER_KD = 0.01;
    public static double SHOOTER_KV = 0.00175;

    public static double KICKER_POWER = 0.5;
    public static int MIN_Hood=850;
      }
    
      private final SparkFlex motor = new SparkFlex(Constants.MOTOR_ID, SparkFlex.MotorType.kBrushless);
      private final SparkFlex motorTwo = new SparkFlex(Constants.MOTOR_TWO_ID, SparkFlex.MotorType.kBrushless);
      private final SparkFlex motorKicker = new SparkFlex(Constants.MOTOR_KICKER_ID, SparkFlex.MotorType.kBrushless);
       private SparkMax motorPreKicker = new SparkMax(Constants.MOTOR_PREKICKER_ID, SparkMax.MotorType.kBrushless);
      private SparkFlexConfig motorConfig = new SparkFlexConfig();
      private SparkFlexConfig motorFollowerConfig = new SparkFlexConfig();
      private SparkFlexConfig motorKickerConfig = new SparkFlexConfig();
      private RelativeEncoder shooterRelativeEncoder = motor.getEncoder();
      ServoHub servoHub = new ServoHub(Constants.SERVO_HUB_ID);
      ServoChannel channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);
      ServoChannel channel1 = servoHub.getServoChannel(ChannelId.kChannelId1);
      private SparkClosedLoopController closedLoopController = motor.getClosedLoopController();;
      private SparkClosedLoopController closedLoopControllerFollower = motorTwo.getClosedLoopController();
      private DoubleEntry flyWheelSpeedNT;
      private DoubleEntry hoodAngleNT;
      private DoubleEntry kickerNT;
      private DoubleEntry flywheelmeasuredspeed;
      private InterpolatingDoubleTreeMap flyWheelCalc = new InterpolatingDoubleTreeMap();
      private InterpolatingDoubleTreeMap hoodCalc = new InterpolatingDoubleTreeMap();
    
      public Shooter() {
    
        configureMotors();
    
        ServoHubConfig config = new ServoHubConfig();
        config.channel0.pulseRange(500, 1500, 2500)
            .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    
        channel0.setPowered(true);
        channel1.setPowered(true);
        channel0.setEnabled(true);
        channel1.setEnabled(true);
    
        // Persist parameters and reset any not explicitly set above to
        // their defaults.
        servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
    
        flyWheelCalc.put(Units.feetToMeters(6), 3000.0);
        flyWheelCalc.put(Units.feetToMeters(10), 3300.0);
        flyWheelCalc.put(Units.feetToMeters(14), 3600.0);
        flyWheelCalc.put(Units.feetToMeters(18), 3900.0);
        flyWheelCalc.put(Units.feetToMeters(22), 4300.0);
    
        // flyWheelCalc.put(0.,0.);
        // flyWheelCalc.put(0.,0.);
        hoodCalc.put(Units.feetToMeters(6), 900.00);
        hoodCalc.put(Units.feetToMeters(10), 1000.);
        hoodCalc.put(Units.feetToMeters(14), 1300.);
        hoodCalc.put(Units.feetToMeters(18), 1500.);
        hoodCalc.put(Units.feetToMeters(22), 1750.);
    
        NetworkTable table = NetworkTableInstance.getDefault().getTable("shooter");
        flyWheelSpeedNT = table.getDoubleTopic("flyWheelSpeed").getEntry(0);
        flyWheelSpeedNT.set(0);
        hoodAngleNT = table.getDoubleTopic("hoodAngle").getEntry(0);
        hoodAngleNT.set(0);
        kickerNT = table.getDoubleTopic("kicker").getEntry(0);
        kickerNT.set(0);
        flywheelmeasuredspeed = table.getDoubleTopic("flyWheelSpeed measured").getEntry(0);
        flywheelmeasuredspeed.set(0);
      }
    
      private void setShooterPower(double power) {
        motor.setVoltage(power);
        motorTwo.setVoltage(power);// Set the motor to the desired power
      }
    
      private void setKickerPower(double power) {
        motorKicker.set(power);
    
      }
    
      private void setShooterSpeed(double speed) {
        closedLoopController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        closedLoopControllerFollower.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      }
    
      private void setServo(int pulse) {
        channel1.setPulseWidth(pulse);
        channel0.setPulseWidth(3000 - pulse);
      }
    
      private void stop() {
        setShooterPower(0);
        setKickerPower(0);
        setServo(850);
    
      }
    
      /*
       * private Command PowerCommand(DoubleSupplier power) {
       * return new FunctionalCommand(
       * () -> {
       * }, // No initialization needed
       * () -> setPower(power.getAsDouble()), // Set intake motor power dynamically
       * interrupted -> stop(), // Stop motor if interrupted
       * () -> false, // Runs indefinitely
       * this // Pass the subsystem
       * );
       * }
       */
    
      private boolean atMinSpeed(double speed) {
        return speed <= shooterRelativeEncoder.getVelocity();
      }
    
      private double distanceToTarget(Pose2d robotPose2d, Pose2d targetPose) {
    
        double X = targetPose.getX() - robotPose2d.getX();
        double Y = targetPose.getY() - robotPose2d.getY();
        return Math.sqrt(X * X + Y * Y);
      }
    
      public Command tuningCommand() {
        return new FunctionalCommand(
            () -> {
            }, // No initialization needed
            () -> {
              setShooterSpeed(flyWheelSpeedNT.get());
              setKickerPower(kickerNT.get());
              setServo((int) hoodAngleNT.get());
            }, // Set intake motor power dynamically
            interrupted -> stop(), // Stop motor if interrupted
            () -> false, // Runs indefinitely
            this // Pass the subsystem
        );
      }
    
      public Command flyWheelSpinUp(double speed, double hoodAngle) { // speed means RPM
        return new FunctionalCommand(
            () -> {
              setShooterPower(1);
              setServo((int)hoodAngle); // pick a hight any hight
        },

        () -> {
          setShooterPower(1);
        },

        interrupted -> {
          stop();

        },

        () -> atMinSpeed(speed),
        this);
  }

  public Command aimbotHub(Supplier<Pose2d> robotPose2d) { // new name?
    return new FunctionalCommand(
        () -> {
          double distance = distanceToTarget(robotPose2d.get(), Constants.hubPose);
          double hoodpulse = hoodCalc.get(distance);
          setShooterSpeed(flyWheelCalc.get(distance));
          setServo((int) hoodpulse);
          setKickerPower(Constants.KICKER_POWER);

        },

        () -> {
          double distance = distanceToTarget(robotPose2d.get(), Constants.hubPose);
          double hoodpulse = hoodCalc.get(distance);
          setShooterSpeed(flyWheelCalc.get(distance));
          setServo((int) hoodpulse);
          setKickerPower(Constants.KICKER_POWER);
        },

        interrupted -> {
          stop();

        },

        () -> false,
        this);
  }

  public Command aimbotDistance(DoubleSupplier distance) { // new name?
    return new FunctionalCommand(
        () -> {

          double hoodpulse = hoodCalc.get(distance.getAsDouble());
          setShooterSpeed(flyWheelCalc.get(distance.getAsDouble()));
          setServo((int) hoodpulse);
          setKickerPower(Constants.KICKER_POWER);

        },

        () -> {

          double hoodpulse = hoodCalc.get(distance.getAsDouble());
          setShooterSpeed(flyWheelCalc.get(distance.getAsDouble()));
          setServo((int) hoodpulse);
          setKickerPower(Constants.KICKER_POWER);
        },

        interrupted -> {
          stop();

        },

        () -> false,
        this);
  }

  public Command distanceShot(DoubleSupplier distance)
  {
    return flyWheelSpinUp(flyWheelCalc.get(distance.getAsDouble()),  hoodCalc.get(distance.getAsDouble())).andThen(aimbotDistance(distance));
  }

  @Override
  public void periodic() {
    flywheelmeasuredspeed.set(shooterRelativeEncoder.getVelocity());

  }

  private void configureMotors() {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not
    // powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or
    // momentum.
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.CURRENT_LIMIT)
        .inverted(true);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.SHOOTER_KP)
        .i(0)
        .d(Constants.SHOOTER_KD)
        .outputRange(-1, 1)

        .allowedClosedLoopError(0.05, ClosedLoopSlot.kSlot0).feedForward
        .kV(Constants.SHOOTER_KV, ClosedLoopSlot.kSlot0)
        .kS(0.12); // Protects motor hardware by limiting current.
    // Apply the configurations to the motor controllers.
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    motorFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.CURRENT_LIMIT) // Protects motor hardware by limiting current.
        .inverted(false);
    motorFollowerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.SHOOTER_KP)
        .i(0)
        .d(Constants.SHOOTER_KD)
        .outputRange(-1, 1)

        .allowedClosedLoopError(0.05, ClosedLoopSlot.kSlot0).feedForward
        .kV(Constants.SHOOTER_KV, ClosedLoopSlot.kSlot0)
        .kS(0.12); // Protects motor hardware by limiting current.

    motorTwo.configure(motorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //
    motorKickerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.CURRENT_LIMIT)
        .inverted(false);

    motorKicker.configure(motorKickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
