package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.GyroScope;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.claw.IntakeGrabInstant;
import frc.robot.commands.claw.IntakeStop;
import frc.robot.commands.claw.IntakeThrowInstant;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup_PickAndPlaceCube extends SequentialCommandGroup {
    //Variables
    public AutoGroup_PickAndPlaceCube(Drivetrain drivetrain, GyroScope gyro, IntakeSub m_intake, ExtensionSubPID m_extensionMotor, PivotSubPID m_pivotMotor){
        
        System.out.println("AutoGroup_Place");
        //Adding a drivetrain
        //Adding Order of commands

        addCommands(
            new ResetEncoders(drivetrain),
            new IntakeGrabInstant(m_intake),
            new PivotMoveToAngleWait(m_pivotMotor, 9),
            new PivotMoveToAngleWait(m_pivotMotor, 95),
            Commands.parallel(new MoveDistance(drivetrain, 36, false),
                              new ExtenderSetPositionWait(m_extensionMotor, 10)),
            new IntakeThrowInstant(m_intake),
            new WaitCommand(0.5),
            new IntakeStop(m_intake),
            // in case it gets stuck on the platforms
            Commands.deadline(new WaitCommand(3), new MoveDistance(drivetrain, 36, true),
            new ExtenderSetPositionWait(m_extensionMotor, 0)),
            new PivotMoveToAngleWait(m_pivotMotor, 50),
            new PivotMoveToAngleWait(m_pivotMotor, 20),
            new WaitCommand(0.5),
            new PivotMoveToAngleWait(m_pivotMotor, 4)
        );
    }
}