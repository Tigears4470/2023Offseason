// package frc.robot.commands.AutoGroups;
// import frc.robot.subsystems.ExtensionSubPID;
// import frc.robot.subsystems.PivotSubPID;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.claw.ClawClose;
// import frc.robot.commands.claw.ClawMove;
// import frc.robot.commands.claw.ClawOpen;
// import frc.robot.commands.pivot.PivotMoveToAngle;
// import frc.robot.subsystems.ClawSub;
// import frc.robot.subsystems.Drivetrain;


// public class AutoGroup_MiddleDrop extends SequentialCommandGroup {
//     //Variables

//     public AutoGroup_MiddleDrop(Drivetrain m_Drivetrain, PivotSubPID m_pivotMotor, ExtensionSubPID m_extensionMotor, ClawSub m_clawMotor){
//         //Adding Order of commands
//         addCommands(
//             new ClawClose(m_clawMotor),
//             new PivotMoveToAngle(m_pivotMotor, 90),
//             Commands.race(new PivotMoveToAngle(m_pivotMotor), new ClawOpen(m_clawMotor)),
//             Commands.race(new ClawMove(m_clawMotor), new PivotMoveToAngle(m_pivotMotor, 20))
//         );
//     }
// }


                                                                           
