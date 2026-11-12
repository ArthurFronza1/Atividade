package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.manipuladorSub;

/**
 * Registra os comandos nomeados usados pelos autônomos do PathPlanner.
 * Reaproveita exatamente o que os botões já fazem:
 * A = intake.coletarAlgae() + intake.transferirAlgae()
 * B = manipulador.arremessarAlgae(intake)
 */
public final class autonomoCMD {

    private autonomoCMD() {
    }

    public static void registrar(
            intakeSub intake,
            elevadorSub elevador,
            manipuladorSub manipulador) {
        Set<Subsystem> req = Set.of(intake, elevador, manipulador);

        // defer = cria um comando novo a cada uso (o mesmo nome aparece várias vezes no
        // auto)
        NamedCommands.registerCommand("ColetarAlga",
                Commands.defer(() -> coletar(intake), req));

        NamedCommands.registerCommand("PontuarNet",
                Commands.defer(() -> arremessar(intake, manipulador), req));

        // Por enquanto igual ao NET; se o processor precisar de outro movimento,
        // é aqui que ele muda.
        NamedCommands.registerCommand("PontuarProcessor",
                Commands.defer(() -> arremessar(intake, manipulador), req));
    }

    private static Command coletar(intakeSub intake) {
        return Commands.sequence(
                // Mesmo efeito do botão Y: libera o intake para uma nova coleta
                Commands.runOnce(
                        () -> intake.definirPosicaoalgae(intakeSub.posicaoalgae.FORA),
                        intake),
                intake.coletarAlgae(),
                intake.transferirAlgae()).withTimeout(5.0);
    }

    private static Command arremessar(intakeSub intake, manipuladorSub manipulador) {
        return manipulador.arremessarAlgae(intake).withTimeout(10.0);
    }
}