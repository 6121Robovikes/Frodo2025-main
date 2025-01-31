package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightSubsytem extends SubsystemBase {
  private RawFiducial[] fiducials;

  public LimelightSubsytem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {
    LimelightHelpers.setCameraPose_RobotSpace(
      "",
      Meters.convertFrom(12.75, Inches),
      0,
      0.195,
      0,
      0,
      0);
      LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {1,4});
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
  }

  public RawFiducial getFiducialWithId(int id) {
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id != id) {
        continue;
      }

      return fiducial;
    }
    throw new NoSuchTargetException("No target with ID " + id + " is in view!");
  }
}







// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

//tanner spelled subsystem wrong lol :P
public class LimelightSubsytem extends SubsystemBase {
  private RawFiducial[] fiducials; 

  private final double limelightMountAngle = 0; //TODO change this, is measured in radians
  private final double limelightMountHeight = 16.5; //TODO change this, is measured in in
  public double x;
  public double y;
  public double Area;
  public double Tid;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty; 
  
  public static final String LIMELIGHT = "limelight";
  //public static final double SHOOTER_POSITION = .5;
  //adjustable transform for the limelight pose per-alliance
  //private static final Transform2d LL_BLUE_TRANSFORM = new Transform2d(0, 0, new Rotation2d());


  //returns info about the april tag (x pos, y pos, screen area, and id)
  public LimelightSubsytem() {
    config(); 
  }

  //RA == Red Alliance, BA == Blue Alliance
  //1 - RA Far side Coral Station
  //2 - RA Processor side Coral Station
  //3 - RA Processor
  //4 - RA Processor side Barge
  //5 - RA Fst side Barge
  //6 - RA Far side reef 1
  //7 to 11 - RA Reef
  //12 - BA Processor side Coral Station
  //13 - BA Far side Coral Station
  //14 - BA Far side Barge
  //15 - BA Processor side Barge
  //16 - BA Processor
  //17 to 22 - BA Reef

  public void config() {

    //height and width should be between 0 and 1
    double windowHeight = 0.55;
    double windowWidth = 0.55;
    double windowYOffset = 0;

    LimelightHelpers.setCropWindow("", -windowWidth, windowWidth, 
    (-windowHeight) + windowYOffset, (windowHeight) + windowYOffset);

    LimelightHelpers.setCameraPose_RobotSpace(
      "",
      Meters.convertFrom(12.75, Inches),
      0,
      0.195,
      0,
      0,
      0);
      LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {1, 4});
  }

  @Override
  public void periodic(){
    fiducials = LimelightHelpers.getRawFiducials("");
  }

    public void publishToDashboard() {
          //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", Area);
    SmartDashboard.putNumber("LimelightTid", Tid);
    }

public void update() 
{
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
    //NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry ty = table.getEntry("ty");
    //NetworkTableEntry ta = table.getEntry("ta");
    //NetworkTableEntry tid = table.getEntry("tid");
    //publishToDashboard();

    //read values periodically
    //double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    //double Area = ta.getDouble(0.0);
    //double Tid = tid.getDouble(0.0);
}

/**Returns a raw fiducial given it's id */
/* 
public RawFiducial getFiducialWithId(int id) 
{
  for (RawFiducial fiducial : fiducials) 
  {
    if (fiducial.id != id) 
    {
      continue; 
    }
    
    return fiducial; 
  }

  throw new NoSuchTargetException("No target with ID " + id + " is in view!");
}

public static class NoSuchTargetException extends RuntimeException {
  public NoSuchTargetException(String message) {
    super(message);
  }
}

public double distanceFromTarget(double targetHeight) {
  return (targetHeight - limelightMountHeight) / Math.tan(y + limelightMountAngle);
}

}*/