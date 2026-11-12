package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSub extends SubsystemBase {

    public enum posicaoalgae {
        FORA,
        CENTRO_INTAKE,
        MANIPULADOR,
        ARREMESSADA
    }

    private posicaoalgae posicaoAlgae = posicaoalgae.FORA;
    private boolean aberto = false;

    public intakeSub() {
    }

    // Altera a posição da Algae
    public void definirPosicaoalgae(posicaoalgae posicao) {
        posicaoAlgae = posicao;
    }

    // Retorna a posição atual da Algae
    public posicaoalgae getPosicaoAlgae() {
        return posicaoAlgae;
    }

    // Verifica se a Algae está no centro do intake
    public boolean temAlgaeNoCentro() {
        return posicaoAlgae == posicaoalgae.CENTRO_INTAKE;
    }

    // Verifica se a Algae está no manipulador
    public boolean temAlgaeNoManipulador() {
        return posicaoAlgae == posicaoalgae.MANIPULADOR;
    }

    // Abrir / fechar intake
    public void abrir() {
        aberto = true;
    }

    public void fechar() {
        aberto = false;
    }

    public boolean estaAberto() {
        return aberto;
    }

    // Comando: alterna entre aberto e fechado
    public Command alternarIntake() {
        return Commands.runOnce(() -> aberto = !aberto, this);
    }

    // Comando: abre o intake, coleta a algae até o centro e fecha
    public Command coletarAlgae() {
        return Commands.sequence(
                Commands.runOnce(this::abrir, this),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                        () -> definirPosicaoalgae(posicaoalgae.CENTRO_INTAKE),
                        this),
                Commands.runOnce(this::fechar, this));
    }

    // Simula a transferência para o manipulador
    public Command transferirAlgae() {
        return Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                        () -> definirPosicaoalgae(posicaoalgae.MANIPULADOR),
                        this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/PosicaoAlgae", posicaoAlgae.toString());
        SmartDashboard.putBoolean("Intake/Aberto", aberto);
        SmartDashboard.putBoolean("Intake/AlgaeNoCentro", temAlgaeNoCentro());
        SmartDashboard.putBoolean("Intake/AlgaeNoManipulador", temAlgaeNoManipulador());
    }
}