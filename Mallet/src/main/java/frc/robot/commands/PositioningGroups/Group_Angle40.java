package frc.robot.commands.PositioningGroups;

import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import frc.robot.commands.extend.ExtenderSetPosition;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Group_Angle40 extends SequentialCommandGroup {

    // ext -6.181
    // pivot 37.228
    // Variables
    public Group_Angle40(ExtensionSubPID extend, PivotSubPID pivot) {
        // Adding Order of commands
        if (pivot.getEncoder().getPosition() < 50) {
            addCommands(
                    new PivotMoveToAngleWait(pivot, 40),
                    new ExtenderSetPosition(extend, 0));
        } else {
            addCommands(
                    new ExtenderSetPositionWait(extend, 0),
                    new PivotMoveToAngleWait(pivot, 40));
        }
    }
}