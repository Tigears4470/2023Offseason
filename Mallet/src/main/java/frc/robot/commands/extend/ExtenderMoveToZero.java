package frc.robot.commands.extend;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubPID;

public class ExtenderMoveToZero extends CommandBase {
    // Required Subsystems
    private ExtensionSubPID m_extender;

    // Creation Function of the Class
    public ExtenderMoveToZero(ExtensionSubPID ext) {
        m_extender = ext;
        addRequirements(m_extender);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    // Tells the extender motor to either move to it's angle or stablize
    @Override
    public void execute() {
        m_extender.setPosition(0);
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
        return true;
    }
}