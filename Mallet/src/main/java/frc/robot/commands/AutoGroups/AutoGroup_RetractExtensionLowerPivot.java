package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.PivotSubPID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup_RetractExtensionLowerPivot extends SequentialCommandGroup {
    // Variables
    public AutoGroup_RetractExtensionLowerPivot(ExtensionSubPID m_extensionMotor, PivotSubPID m_pivotMotor) {
        addRequirements(m_extensionMotor, m_pivotMotor);
        addCommands(
                new AutoGroup_RetractExtension(m_extensionMotor),
                new AutoGroup_LowerPivot(m_pivotMotor));
    }
}