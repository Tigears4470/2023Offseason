package frc.robot.commands.PositioningGroups;

import frc.robot.commands.extend.ExtenderSetPositionWait;
import frc.robot.commands.pivot.PivotMoveToAngle;
import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.PivotSubPID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Group_RetractAll extends SequentialCommandGroup {

    public Group_RetractAll(PivotSubPID m_pivotMotor, ExtensionSubPID m_extensionMotor) {

        addCommands(
                new ExtenderSetPositionWait(m_extensionMotor, 0),
                new PivotMoveToAngle(m_pivotMotor, 0));
    }
}