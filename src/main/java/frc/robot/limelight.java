// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase {

  public boolean tv;
  public double TX;
  public double distance;
  public boolean isBlue=true;
  /** Creates a new limelight. */
  public limelight() {    

  }

  public double getTx()
  {
    return TX;
  }

  public double getDistance()
  {
    return distance;
  }

  public boolean getTv()
  {
    return tv;
  }

  @Override
  public void periodic() {
    if (!DriverStation.getAlliance().isEmpty()&&DriverStation.isDisabled())
    {
      isBlue=DriverStation.getAlliance().get()==DriverStation.Alliance.Blue;
      if (isBlue) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        
      }
      else
      {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      }
    }

    tv=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)==1.0;
    TX=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    distance=NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2];
    
  }
}
