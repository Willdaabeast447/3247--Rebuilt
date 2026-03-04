// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  
  private class Constants
  {
    //Motor ID numbers
    private final static int CLIME_MOTOR_ID = 51;
    private final static int DEPLOY_MOTOR_ID = 50;
    //Spark Max current limit
    private final static int MAX_CURRENT_LIMIT = 40;
    //Spark Flex current limit
    private final static int FLEX_CURRENT_LIMIT = 80;
    public static double OUT_POSITION=152;
    public static double DOWN_POSITION=1;
    public static double CLIMBER_TOP= 0;
    public static double CLIMBER_L1= 0;
    public static double CLIMBER_BOTTOM= 0;
    //the position when its safe for the climer to start going up
    public final static double SAFE_POS = 50;
  }
        
  private final SparkFlex climbMotor = new SparkFlex(Constants.CLIME_MOTOR_ID,SparkFlex.MotorType.kBrushless);
  private final SparkMax mastMotor = new SparkMax(Constants.DEPLOY_MOTOR_ID,SparkMax.MotorType.kBrushless);
  private SparkFlexConfig climeConfig = new SparkFlexConfig();
  private SparkMaxConfig mastConfig = new SparkMaxConfig();
  private RelativeEncoder mastEncoder= mastMotor.getEncoder();
  private RelativeEncoder climberEncoder= climbMotor.getEncoder();
  private boolean isRetracted=true;
  private SparkClosedLoopController mastLoopController;
  private SparkClosedLoopController climbLoopController;
  

  public Climber() 
  {
    configureMotors();
    mastLoopController=mastMotor.getClosedLoopController();
    climbLoopController=climbMotor.getClosedLoopController();

  }

  
    private void configureMotors() 
  {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or momentum.
    mastConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MAX_CURRENT_LIMIT) // Protects motor hardware by limiting current.
        .inverted(true);
    mastConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0025)
        .i(0)
        .d(0)
        .outputRange(-1, 1)        
        .allowedClosedLoopError(0.05, ClosedLoopSlot.kSlot0)
        .feedForward
        .kS(0.6);
    mastConfig.softLimit
        .forwardSoftLimit(5.16)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    // Apply the configurations to the motor controllers.
    mastMotor.configure(mastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.FLEX_CURRENT_LIMIT)
        .inverted(true); // Protects motor hardware by limiting current.

    climeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.015)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)
        .feedForward
        .kS(0.23);
    climeConfig.softLimit
        .forwardSoftLimit(146.1)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    // Apply the configurations to the motor controllers.
    climbMotor.configure(climeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setMastPosition(double position)
{
  mastLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
} 

private void setClimberPosition(double position)
{
  climbLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
private void stop(){
  mastMotor.set(0);
  climbMotor.set(0);
} 





public boolean isRetracted(){
  return isRetracted;
}

    
  public Command DepolyCommand() {
    return new FunctionalCommand(
      () -> {}, // No initialization needed
      () -> {
        setMastPosition(Constants.OUT_POSITION);
        if (mastEncoder.getPosition()>Constants.SAFE_POS)
              {
               setClimberPosition(Constants.CLIMBER_TOP); 
              }
        else{
            setClimberPosition(Constants.CLIMBER_BOTTOM);
        }
      }, // Set intake motor power dynamically
      interrupted -> {}, // Stop motor if interrupted
      () -> {return (mastEncoder.getPosition()>150&&climberEncoder.getPosition()>=Constants.CLIMBER_TOP);},
      this // Pass the subsystem
      );
    }

    public Command L1Command() {
    return new FunctionalCommand(
      () -> {}, // No initialization needed
      () -> {
        setMastPosition(Constants.OUT_POSITION);
        if (mastEncoder.getPosition()>Constants.SAFE_POS)
              {
               setClimberPosition(Constants.CLIMBER_L1);
              }
        else{
            setClimberPosition(Constants.CLIMBER_TOP);
        }
      }, // Set intake motor power dynamically
      interrupted -> {}, // Stop motor if interrupted
      () -> false,
      this // Pass the subsystem
      );
    }

     public Command LetGoCommand() {
    return new FunctionalCommand(
      () -> {}, // No initialization needed
      () -> {
        setMastPosition(Constants.OUT_POSITION);
        setClimberPosition(Constants.CLIMBER_TOP); 
      }, // Set intake motor power dynamically
      interrupted -> {}, // Stop motor if interrupted
      () -> false,
      this // Pass the subsystem
      );
    }



public Command RetractCommand() {
 return new FunctionalCommand(
      () -> {}, // No initialization needed
      () -> {
        setClimberPosition(Constants.CLIMBER_BOTTOM);
        if (climberEncoder.getPosition()<Constants.SAFE_POS)
              {
               setMastPosition(Constants.DOWN_POSITION); 
              }
        else{
            setMastPosition(Constants.SAFE_POS);
        }
      }, // Set intake motor power dynamically
      interrupted -> {}, // Stop motor if interrupted
      () -> {return (mastEncoder.getPosition()<10&&climberEncoder.getPosition()<=Constants.CLIMBER_BOTTOM);},
      this // Pass the subsystem
      );
}

public Command manual(DoubleSupplier mastpower,DoubleSupplier climbePower)
{
  return new FunctionalCommand(
      () -> {}, // No initialization needed
      () -> {
        climbMotor.set(climbePower.getAsDouble()*0.25);
        mastMotor.set(mastpower.getAsDouble()*0.25);        
      }, // Set intake motor power dynamically
      interrupted -> {stop();}, // Stop motor if interrupted
      () -> false,
      this // Pass the subsystem
      );
}

  @Override
  public void periodic() {

    isRetracted= mastEncoder.getPosition()<10;
    // This method will be called once per scheduler run
  }
}
