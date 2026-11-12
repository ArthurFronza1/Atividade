
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.manipuladorSub;

public class TransferirCMD extends SequentialCommandGroup {

        public TransferirCMD(
                        intakeSub intake,
                        elevadorSub elevador,
                        manipuladorSub manipulador) {
                addCommands(
                                Commands.either(
                                                Commands.sequence( 
                                                                intake.transferirAlgae(),
                                                                new AutomacaoAlgaeCMD(elevador, manipulador)),

                                                Commands.none(),

                                                intake::temAlgaeNoCentro));
        }

}