package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.GyroScope;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AngleAutoBalance extends CommandBase {
  private final GyroScope m_gyro;
  private final Drivetrain m_drivetrain;
  private boolean isBalancing;
  private boolean isBackwards;
  private int backwardsScaler;
  
  /**
   * Creates a new AngleAutoBalance. This command balances the robot on the charging station. 
   * THIS WILL ONLY WORK IF THE ROBOT'S X ANGLE STARTS OFF POSITIVE (IMU orientation)
   * This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param gyro The gyro subsystem on which this command will run
   */
  public AngleAutoBalance(Drivetrain drivetrain, GyroScope gyro, boolean backwards) {
    isBalancing = false;

    // if starting from other side of charger, go backwards and change which side is other side
    backwardsScaler = backwards ? -1 : 1; 
    isBackwards = backwards;
    m_gyro = gyro;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("initializing");
    SmartDashboard.putBoolean("Balancing", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getAngleX();
    pitchAngleDegrees+=m_gyro.getAngleX();
    pitchAngleDegrees+=m_gyro.getAngleX();
    pitchAngleDegrees/=3.0; 
    if (!isBalancing)
    {
      m_drivetrain.arcadeDrive(backwardsScaler*.5, 0); // competition .34
      // if at high enough angle so we know it is on the platform
      if (Math.abs(pitchAngleDegrees) > Constants.K_PLAT_DEGREE_THRESH)
        isBalancing = true;
    }
    else {
      if(Math.abs(pitchAngleDegrees) > 2) {
        double rate = Math.signum(pitchAngleDegrees)*Math.min((.003*Math.pow(pitchAngleDegrees, 2.0)+0.1),0.5); // slightly parabolic better than linear
        m_drivetrain.arcadeDrive(rate, 0);
      } else {
        m_drivetrain.arcadeDrive(0, 0);
      }
      SmartDashboard.putBoolean("Balancing", true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't end, stay balanced until autonomous ends
    return false;
  }
}
