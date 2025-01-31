package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsytem;

public class AimCommand extends Command 
{
    private LimelightSubsytem limeLight;
    private CommandSwerveDrivetrain drivetrain;

    private static final PIDController rotationalPidController = new PIDController(0.05, 0, 0, 0.5);

    private static final PIDController xPidController = new PIDController(0.1, 0, 0, 0.2);
    private static final PIDController yPidController = new PIDController(0.1, 0, 0, 0.5);

    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric();
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private int id = 1;


    /** The Aim Command, the aimster, the almighty aimer, the aimiest of them all, the AIM OVERLORD, the one aim to rule them all */
    public AimCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsytem limelight, int fiducialId) 
    {
        this.drivetrain = drivetrain;
        this.limeLight = limelight;
        this.id = fiducialId;
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

            final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * 0.75 * 0.5;
            final double velocityX = xPidController.calculate(fiducial.distToRobot, 2.25) * -1.0 * 4.73* 0.5 ;
            

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
