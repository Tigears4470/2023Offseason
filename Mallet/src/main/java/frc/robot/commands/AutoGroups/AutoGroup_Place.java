package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.GyroScope;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.claw.IntakeGrabInstant;
import frc.robot.commands.claw.IntakeStop;
import frc.robot.commands.claw.IntakeThrowInstant;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup_Place extends SequentialCommandGroup {
    //Variables
    public AutoGroup_Place(Drivetrain drivetrain, GyroScope gyro, IntakeSub m_intake, ExtensionSubPID m_extensionMotor, PivotSubPID m_pivotMotor){
        
        System.out.println("AutoGroup_Place");
        //Adding a drivetrain
        //Adding Order of commands

        addCommands(
            new ResetEncoders(drivetrain),
            // new MoveDistance(drivetrain, 1, false),
            // new TurnBy(drivetrain, gyro, 180),
            // new MoveDistance(drivetrain, 0, false)
            new IntakeGrabInstant(m_intake),
            new PivotMoveToAngleWait(m_pivotMotor, 85),
            new ExtenderSetPositionWait(m_extensionMotor, 5),
            new IntakeThrowInstant(m_intake),
            new WaitCommand(1),
            new IntakeStop(m_intake)
        );
    }
}