package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup_MaxPivot extends SequentialCommandGroup {
    // Variables
    public AutoGroup_MaxPivot(PivotSubPID m_pivotMotor) {
        addRequirements(m_pivotMotor);
        addCommands(
                new PivotMoveToAngleWait(m_pivotMotor, 95));
    }
}