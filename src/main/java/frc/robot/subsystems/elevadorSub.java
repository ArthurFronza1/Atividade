
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class elevadorSub extends SubsystemBase {

    private double alturaAtual = 0.0;
    private double alturaDesejada = 0.0;

    public elevadorSub() {

    }

    public void definirAltura(double altura) {
        alturaDesejada = Math.max(0.0, altura);
    }

    public double getAltura() {
        return alturaAtual;
    }

    public double getAlturaDesejada() {
        return alturaDesejada;
    }

    public boolean estaNaAltura(double altura) {
        return Math.abs(getAltura() - altura) <= Constants.Elevador.TOLERANCIA;
    }

    public boolean estaNaColeta() {
        return estaNaAltura(Constants.Elevador.ALTURA_COLETA);
    }

    public boolean estaNoZero() {
        return estaNaAltura(0.0);
    }

    // Comando para movimentar o elevador até uma altura
    public Command irParaAltura(double altura) {
        return Commands.run(
                () -> definirAltura(altura),
                this).until(() -> estaNaAltura(altura));
    }

    @Override
    public void periodic() {

        double diferenca = alturaDesejada - alturaAtual;

        double variacaoMaxima = Constants.Elevador.VELOCIDADE_MAX
                * Constants.Simulacao.PERIOD;

        double variacao = MathUtil.clamp(
                diferenca,
                -variacaoMaxima,
                variacaoMaxima);

        alturaAtual += variacao;

        SmartDashboard.putNumber(
                "Elevador/Altura",
                alturaAtual);

        SmartDashboard.putNumber(
                "Elevador/AlturaDesejada",
                alturaDesejada);

        SmartDashboard.putBoolean(
                "Elevador/NaColeta",
                estaNaColeta());
    }
}
