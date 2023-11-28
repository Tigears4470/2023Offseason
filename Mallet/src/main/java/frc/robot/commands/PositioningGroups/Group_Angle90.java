package frc.robot.commands.PositioningGroups;

import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.pivot.PivotMoveToAngle;
import frc.robot.commands.extend.ExtenderSetPosition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Group_Angle90 extends SequentialCommandGroup {
    // Variables
    public Group_Angle90(ExtensionSubPID extend, PivotSubPID pivot) {
        // Adding a drivetrain
        // Adding Order of commands
        if (pivot.getEncoder().getPosition() < 45) {
            addCommands(
                    new PivotMoveToAngle(pivot, 90),
                    new ExtenderSetPosition(extend, 0));
        } else {
            addCommands(
                    new ExtenderSetPosition(extend, 0),
                    new PivotMoveToAngle(pivot, 90));
        }
    }
}