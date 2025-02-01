package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsytem;

public class AimCommand extends Command 
{

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEG_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
        new Translation3d(1.5, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI)
      );


    private final LimelightSubsytem limeLight;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> poseProvider;

    private static final PIDController rotationalPidController = new PIDController(0.05, 0, 0, 0.5);

    private static final PIDController xPidController = new PIDController(0.1, 0, 0, 0.2);
    private static final PIDController yPidController = new PIDController(0.1, 0, 0, 0.5);
    private static final PIDController omegaPidController = new PIDController(0.1, 0, 0, 0.5);

    

    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric();
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    //the fiducial id we a currently looking at
    private int id = 1;

    public AimCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsytem limelight, int fiducialId, Supplier<Pose2d> poseProvider) 
    {
        this.drivetrain = drivetrain;
        this.limeLight = limelight;
        this.id = fiducialId;
        this.poseProvider = poseProvider;

        xPidController.setTolerance(.02);
        yPidController.setTolerance(.02);
        omegaPidController.setTolerance(.02);
    }
    
    @Override public void initialize() 
    {
        
    }

    @Override public void execute() 
    {
        RawFiducial fiducial;

        try 
        {
            fiducial = limeLight.getFiducialWithId(id);
            //setpoint = speed
            final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * 0.75 * 0.5;
            final double velocityX = xPidController.calculate(fiducial.distToRobot, 5) * -1.0 * 4.73* 0.5;

            if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) 
            {
                this.end(true);
            }

            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalPidController", rotationalRate);
            SmartDashboard.putNumber("xPidController", velocityX);
            drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));

        } 
        catch (LimelightSubsytem.NoSuchTargetException nste) 
        {

        }
  }

  /** Only returns false right now for some reason */
  @Override public boolean isFinished() 
  {
    return false;
  }

  @Override public void end(boolean interrupted) 
  {
    Supplier<SwerveRequest> swerveSupplier = () -> idleRequest;
    drivetrain.applyRequest(swerveSupplier);
  }
}
