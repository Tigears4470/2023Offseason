package frc.robot.commands.AutoGroups;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtensionSubPID;
import frc.robot.subsystems.GyroScope;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.PivotSubPID;
import frc.robot.Constants;
import frc.robot.commands.AngleAutoBalance;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.MoveDistanceFast;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.claw.IntakeGrabInstant;
import frc.robot.commands.claw.IntakeStop;
import frc.robot.commands.claw.IntakeThrowInstant;
import frc.robot.commands.extend.ExtenderSetPositionWait;
import frc.robot.commands.pivot.PivotMoveToAngleWait;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGroup_PlaceCubeHighLeaveBalance extends SequentialCommandGroup {
    //Variables
    public AutoGroup_PlaceCubeHighLeaveBalance(Drivetrain drivetrain, GyroScope gyro, IntakeSub m_intake, ExtensionSubPID m_extensionMotor, PivotSubPID m_pivotMotor){
        
        System.out.println("AutoGroup_PlaceCubeHighThenBalance");
        //Adding a drivetrain
        //Adding Order of commands

        addCommands(
            new ResetEncoders(drivetrain),
            new IntakeGrabInstant(m_intake),
            new PivotMoveToAngleWait(m_pivotMotor, 9),
            new PivotMoveToAngleWait(m_pivotMotor, Constants.K_ANGLE_AUTO_HIGH),
            Commands.parallel(Commands.race(new MoveDistanceFast(drivetrain, 1.5*12, false), new WaitCommand(3.5)),
                              new ExtenderSetPositionWait(m_extensionMotor, Constants.K_EXT_AUTO_HIGH)),
            new IntakeThrowInstant(m_intake),
            new WaitCommand(0.2),
            new IntakeStop(m_intake),
            // in case it gets stuck on the platforms
            Commands.parallel(new MoveDistanceFast(drivetrain, 15*12, true),
            new AutoGroup_RetractExtension(m_extensionMotor).andThen(new AutoGroup_LowerPivot(m_pivotMotor))),
            new AngleAutoBalance(drivetrain, gyro, false)
        );
    }
}