
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.intakeSub;

public class coletaCMD {

    public static Command criarColeta(intakeSub intake) {

        return Commands.either(

                Commands.sequence(
                        Commands.waitSeconds(1.0),

                        Commands.runOnce(
                                () -> intake.definirPosicaoalgae(
                                        intakeSub.posicaoalgae.CENTRO_INTAKE),
                                intake)),

                Commands.none(),

                () -> intake.getPosicaoAlgae() == intakeSub.posicaoalgae.FORA);
    }
}
