// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

private class Constants
  {
  //Motor ID numbers
  private final static int INTAKE_MOTOR_ID = 31;
  private final static int DEPLOY_MOTOR_ID = 30;
  //Spark Max current limit
  private final static int MAX_CURRENT_LIMIT = 40;
  //Spark Flex current limit- not used just incase of brown outs
  private final static int FLEX_CURRENT_LIMIT = 40;
  // The positon on the incoder that it's at when up and down
  private final static double UP_POSITION = 0.24;
  private final static double DOWN_POSITION = 0.06;
  //accept able range of how close to down position we need to be
  private final static double DOWN_RANGE = 0.1;
  //accept able range of how close to up position we need to be
  private final static double UP_RANGE = 0.25;
  }
public boolean isUp = true;//true means up 
private final SparkFlex intakeMotor = new SparkFlex(Constants.INTAKE_MOTOR_ID,SparkFlex.MotorType.kBrushless);
private final SparkMax deployMotor = new SparkMax(Constants.DEPLOY_MOTOR_ID,SparkMax.MotorType.kBrushless);
private SparkMaxConfig deployConfig = new SparkMaxConfig();
private SparkFlexConfig intakeConfig = new SparkFlexConfig();
private final SparkAbsoluteEncoder encoder = deployMotor.getAbsoluteEncoder();
private SparkClosedLoopController deployLoopController;
//private SparkClosedLoopController intakeLoopController; unused at current moment
private DoubleEntry deployNT;
private DoubleEntry intakeNT;
private double holdAngle;
public Intake() 
  {
   configureMotors();
   deployLoopController = deployMotor.getClosedLoopController();
   //intakeLoopController = intakeMotor.getClosedLoopController(); unused at current moment
   NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
   deployNT = table.getDoubleTopic("deploy").getEntry(0);
   deployNT.set(0);
   intakeNT = table.getDoubleTopic("intakeSpeed").getEntry(0);
   intakeNT.set(0);

  }
private void configureMotors() 
  {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or momentum.
    deployConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MAX_CURRENT_LIMIT)
        .inverted(true); // Protects motor hardware by limiting current.
    // Apply the configurations to the motor controllers.
   
        deployConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(2.5)
        .i(0)
        .d(10)
        .outputRange(-1, 1)
      
        .allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0)
        .feedForward
        .kS(0.675)
        .kCos(0.975);


    deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80);
}

private void setPosition(double position)
{
  deployLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
} 

private void setDeployPower(double power)
{
  deployMotor.set(power);
}

private void setRollerPower(double power)
{
  intakeMotor.set(power);
}

public void stop()
{
  deployMotor.set(0);
  intakeMotor.set(0);
}

public boolean isUp(){
  return isUp;
}

public Command Hold()
{
    return new FunctionalCommand(
      () -> {
        holdAngle=encoder.getPosition();
      }, // No initialization needed
      () -> { 
              setRollerPower(0);
              setPosition(holdAngle);
            }, 
      interrupted -> stop(), // Stop motor if interrupted
      () -> false, // Runs indefinitely
      this // Pass the subsystem
  );
}

public Command intakeCommand() {
  return new FunctionalCommand(
      () -> {
      }, // No initialization needed
      () -> { 
              setPosition(Constants.DOWN_POSITION);// setting this everytime is unnecessary overhead(From Hunter to Will)
              setRollerPower(1);
            }, // Set intake motor power dynamically
      interrupted -> stop(), // Stop motor if interrupted
      () -> false, // Runs indefinitely
      this // Pass the subsystem
  );
}

public Command DeployCommand() {
  return new FunctionalCommand(
      () -> {
      }, // No initialization needed
      () -> {
              setPosition(Constants.DOWN_POSITION);
              setRollerPower(0);
            }, // Set intake motor power dynamically
      interrupted -> stop(), // Stop motor if interrupted
      () -> {return encoder.getPosition()<Constants.DOWN_RANGE;},
      this // Pass the subsystem
  );
}

public Command RetractCommand() {
  return new FunctionalCommand(
      () -> {
      }, // No initialization needed
      () -> {
              setPosition(Constants.UP_POSITION);
              setRollerPower(0);
            }, // Set intake motor power dynamically
      interrupted -> stop(), // Stop motor if interrupted
      () -> {return encoder.getPosition()>Constants.UP_RANGE;},
      this // Pass the subsystem
  );
}

public Command manualCommand()
  {
  return new FunctionalCommand(
        () -> {
        }, // No initialization needed
        () -> {
              {
                setDeployPower(deployNT.get());
                setRollerPower(intakeNT.get());
              }
              
              }, // Set intake motor power dynamically
        interrupted -> stop(), // Stop motor if interrupted
        () -> false, // Runs indefinitely
        this // Pass the subsystem
    );
  }
 
  

@Override
public void periodic() {
  intakeNT.set(encoder.getPosition());
  isUp= encoder.getPosition()>=(Constants.UP_POSITION-0.02);
}


 
  
}
