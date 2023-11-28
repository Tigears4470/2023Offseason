package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.MoveDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup_MoveTest extends SequentialCommandGroup {
    // Variables
    private Drivetrain m_drivetrain;

    public AutoGroup_MoveTest(Drivetrain drivetrain) {
        // Adding a drivetrain
        m_drivetrain = drivetrain;
        // Adding Order of commands
        addCommands(
                new ResetEncoders(m_drivetrain),
                new MoveDistance(m_drivetrain, 36, true),
                new WaitCommand(1),
                new MoveDistance(m_drivetrain, 36, false)
        // new MoveDistanceV2(this.m_Drivetrain, 2, false)
        );
    }
}
