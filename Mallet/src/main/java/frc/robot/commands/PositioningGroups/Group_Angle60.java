package frc.robot.commands.PositioningGroups;

import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Group_Angle60 extends SequentialCommandGroup {
    // Variables
    public Group_Angle60(ExtensionSubPID extend, PivotSubPID pivot) {
        // Adding Order of commands
        addCommands(
                new ExtenderSetPositionWait(extend, 0),
                new PivotMoveToAngleWait(pivot, 60));
    }
}