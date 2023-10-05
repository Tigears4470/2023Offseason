package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroScope;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnBy extends CommandBase {
    private Drivetrain m_drivetrain;
    private GyroScope m_gyro;
    private float turnAmount; //angle we're turning by
    private float finalAngle; //the final angle the robot should end at
    private float turnDirection;

    public TurnBy (Drivetrain p_drivetrain, GyroScope p_gyro, float angle){
        m_drivetrain = p_drivetrain;
        m_gyro = p_gyro;
        turnAmount = angle;
        addRequirements(m_drivetrain);
    }

    public float getGyroZ360(){ //return the gyro position in the range 0 to 359
        float angle = m_gyro.getAngleZ(); //gyro originally returns a value from -180 to 180
        if(angle<0) angle += 360;
        return angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finalAngle = getGyroZ360() + turnAmount; //determine final angle based on current position and the angle to turn by
        finalAngle = (finalAngle+360)%360; // Properly converts final angle to a positive number (ex: 45 degrees and goes counter-clockwise by 90 degrees would be 270 or -90)
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