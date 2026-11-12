package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.manipuladorSub;

public class AutomacaoAlgaeCMD extends SequentialCommandGroup {

        public AutomacaoAlgaeCMD(
                        elevadorSub elevador,
                        manipuladorSub manipulador) {

                addCommands(

                                Commands.either(

                                                // Se já estiver tudo na posição de coleta
                                                Commands.none(),

                                                // Caso contrário, executa a sequência
                                                Commands.sequence(

                                                                // Primeiro retorna o manipulador para 0 graus
                                                                manipulador.irParaAngulo(0),

                                                                // Depois desce o elevador
                                                                elevador.irParaAltura(
                                                                                Constants.Elevador.ALTURA_COLETA)),

                                                // Verifica se os dois já estão na posição
                                                () -> manipulador.estaNoZero()
                                                                && elevador.estaNaColeta()));
        }
}