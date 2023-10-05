package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroScope;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToMatch extends CommandBase {
    private Drivetrain m_drivetrain;
    private GyroScope m_gyro;
    private float finalAngle; //the final angle the robot should end at
    private float turnDirection;

    public TurnToMatch (Drivetrain p_drivetrain, GyroScope p_gyro, float angle){
        m_drivetrain = p_drivetrain;
        m_gyro = p_gyro;
        finalAngle = angle;
        addRequirements(m_drivetrain);
    }

    public float getGyroZ360(){ //return the gyro position in the range 0 to 359
        float angle = m_gyro.getAngleZ();
        if(angle<0) angle += 360.0f;
        return angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turnDirection = Math.signum(finalAngle-getGyroZ360());
        //Goes opposite direction if angle distance is greater than 180
        if(Math.abs(finalAngle - getGyroZ360()) > 180.0f){
            turnDirection *= -1.0f;
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        float distance = Math.abs(finalAngle - getGyroZ360()); //find distance from the target angle
        if(distance > 180) distance = Math.abs(distance-360.0f); //wrap around distance
        float speed = 2.0f*Math.max(Constants.K_MIN_TURNING_SPEED,Math.min(Constants.K_MAX_TURNING_SPEED, (.01f*(distance-10.0f)+.2f))); //set speed dynamically to slow down as we approach the target angle

        m_drivetrain.arcadeDrive(0, speed*turnDirection);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(finalAngle-getGyroZ360()) < Constants.K_TURN_ERROR_RANGE;
    }
}