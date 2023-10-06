package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup_LowerPivot extends SequentialCommandGroup {
    //Variables
    public AutoGroup_LowerPivot(PivotSubPID m_pivotMotor){
        addRequirements(m_pivotMotor);
        addCommands(
            new PivotMoveToAngleWait(m_pivotMotor, 50),
            new PivotMoveToAngleWait(m_pivotMotor, 20),
            new WaitCommand(0.5),
            new PivotMoveToAngleWait(m_pivotMotor, 4)
        );
    }
}