package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveDistanceV2 extends CommandBase{
    private Drivetrain m_Drivetrain;
    private double endDistance;
    private double distance;
    private boolean isBackwards;
    /**
     * @param drivetrain the drivetrain to be used
     * @param distance the desired distance to traveled in inches
     * @param isBackwards the direction to head
     */
    public MoveDistanceV2(Drivetrain drivetrain, double distance, boolean isBackwards){
        this.isBackwards = isBackwards;
        m_Drivetrain  = drivetrain;
        this.distance = distance;
        m_Drivetrain.resetEncoders();
        if(!isBackwards)
            endDistance = m_Drivetrain.getAverageDistanceInch() + this.distance;
        else    
            endDistance = m_Drivetrain.getAverageDistanceInch() - this.distance;
        addRequirements(drivetrain);
    }
    
    public void initialize() {
        m_Drivetrain.resetEncoders();
        if(!isBackwards)
            endDistance = m_Drivetrain.getAverageDistanceInch() + distance;
        else 
            endDistance = m_Drivetrain.getAverageDistanceInch() - distance;
    }

    public void execute() {
        //Moves the drivetrain backwards of forwards
        if(isBackwards)
            m_Drivetrain.arcadeDrive(-.4, 0);
        else   
            m_Drivetrain.arcadeDrive(0.4, 0);
    }
    
    public void end(boolean interrupted){
        //Stops drivetrain
        m_Drivetrain.arcadeDrive(0.0, 0.0);
    }
    public boolean isFinished(){
        //Determines the status of this function with range of error
        //Does not use abs because is endDistane is -1 and we're at 0, could cause a false true 
        if(isBackwards)
            return -1 * endDistance > m_Drivetrain.getAverageDistanceInch() - Constants.K_MOVE_DISTANCE_ERROR_RANGE;
        else
            return endDistance < m_Drivetrain.getAverageDistanceInch() + Constants.K_MOVE_DISTANCE_ERROR_RANGE;
    }
}
