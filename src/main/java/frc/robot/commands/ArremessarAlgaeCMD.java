package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.manipuladorSub;

public class ArremessarAlgaeCMD extends SequentialCommandGroup {

    // Arremesso padrão (o mesmo de antes)
    public ArremessarAlgaeCMD(
            intakeSub intake,
            elevadorSub elevador,
            manipuladorSub manipulador) {
        this(
                intake,
                elevador,
                manipulador,
                Constants.Elevador.ALTURA_LANCAMENTO,
                Constants.Manipulador.ANGULO_LANCAMENTO);
    }

    // Arremesso com altura/ângulo próprios (NET e PROCESSOR podem ser diferentes)
    public ArremessarAlgaeCMD(
            intakeSub intake,
            elevadorSub elevador,
            manipuladorSub manipulador,
            double altura,
            double angulo) {

        addCommands(

                Commands.either(

                        Commands.sequence(

                                // Eleva o elevador
                                elevador.irParaAltura(altura),

                                // Posiciona o manipulador
                                manipulador.irParaAngulo(angulo),

                                // Marca a Algae como arremessada
                                Commands.runOnce(
                                        () -> intake.definirPosicaoalgae(
                                                intakeSub.posicaoalgae.ARREMESSADA),
                                        intake)),

                        Commands.none(),

                        // Só permite arremessar se houver Algae
                        intake::temAlgaeNoManipulador));
    }
}