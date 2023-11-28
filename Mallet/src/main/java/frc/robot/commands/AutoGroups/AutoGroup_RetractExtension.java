package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup_RetractExtension extends SequentialCommandGroup {
    // Variables
    public AutoGroup_RetractExtension(ExtensionSubPID m_extensionMotor) {
        addRequirements(m_extensionMotor);
        addCommands(
                new ExtenderSetPositionWait(m_extensionMotor, 0));
    }
}