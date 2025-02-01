// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsytem;



public class Odometry extends Command
{
  private static final PIDController rotationalPidController = new PIDController(0.05, 0, 0, 0.5);

  private static final double xKp = 0.1;
  private static final PIDController xPidController = new PIDController(xKp, 0, 0, 0.2);
  private static final PIDController yPidController = new PIDController(0.1, 0, 0, 0.5);

  private LimelightSubsytem limeLight;
  private int id = 1;

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {
    RawFiducial fiducial = limeLight.getFiducialWithId(id);
    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    //double kP = 0.035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * xKp;

    // convert to radians per second for our drive method
   // targetingAngularVelocity *= Drivetrain.kMaxAngularSpeed;
    //setpoint = speed
    final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * 0.75 * 0.5;
    final double velocityX = xPidController.calculate(fiducial.distToRobot, 5) * -1.0 * 4.73 * 0.5;
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= TunerConstants.kSlipCurrent.baseUnitMagnitude();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
  /* 
  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.joystick.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.joystick.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.joystick.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(RobotContainer.joystick.a())
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
      }
    
      private Object getPeriod() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPeriod'");
      }
    
      /** Creates a new Limelight2. */
  public Odometry() 
  {

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
