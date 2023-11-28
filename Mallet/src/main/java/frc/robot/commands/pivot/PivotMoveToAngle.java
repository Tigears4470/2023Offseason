package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubPID;

public class PivotMoveToAngle extends CommandBase {
    // Required Subsystems
    private PivotSubPID m_pivot;
    private double m_finalAngle;

    // Creation Function of the Class
    public PivotMoveToAngle(PivotSubPID pivot) {
        m_pivot = pivot;
        addRequirements(m_pivot);
    }

    public PivotMoveToAngle(PivotSubPID pivot, double angle) {
        m_pivot = pivot;
        m_finalAngle = angle;
        addRequirements(m_pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pivot.setAngle(m_finalAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    // Nothing is called here as it is covered already in the subsystem to stop the
    // motor.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Checks to see if the motor is close to the desired angle before ending (for
        // command groups)
        return true;
    }
}
