package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.ClawSub;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.GyroScope;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.TurnBy;
import frc.robot.commands.MoveDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup_PlaceAndLeave extends SequentialCommandGroup {
    // Variables
    private Drivetrain m_drivetrain;
    private GyroScope m_gyro;

    public AutoGroup_PlaceAndLeave(Drivetrain drivetrain, GyroScope gyro, PivotSubPID m_pivot,
            ExtensionSubPID m_extender, ClawSub claw) {
        System.out.println("AutoGroup_PlaceAndLeave");
        // Adding a drivetrain
        m_drivetrain = drivetrain;
        m_gyro = gyro;
        // Adding Order of commands
        addCommands(
                new TurnBy(m_drivetrain, m_gyro, 180f),
                new MoveDistance(drivetrain, 15, false)
        /* move back into community and center with AprilTag */
        /* place piece */
        );
    }
}
