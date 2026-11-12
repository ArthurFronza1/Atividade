package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class manipuladorSub extends SubsystemBase {

    private double anguloAtual = Constants.Manipulador.ANGULO_INICIAL;
    private double setpoint = Constants.Manipulador.ANGULO_INICIAL;

    // Representação visual do braço
    private final Mechanism2d mecanismo = new Mechanism2d(1.0, 1.0);

    private final MechanismRoot2d base = mecanismo.getRoot("Base", 0.5, 0.1);

    private final MechanismLigament2d braco = base.append(
            new MechanismLigament2d(
                    "Braco",
                    0.5,
                    anguloAtual,
                    6,
                    new Color8Bit(Color.kGreen)));

    public manipuladorSub() {
        SmartDashboard.putData("Manipulador/Mecanismo", mecanismo);
    }

    // Define o ângulo desejado
    public void definirAngulo(double angulo) {
        setpoint = MathUtil.clamp(
                angulo,
                Constants.Manipulador.ANGULO_MIN,
                Constants.Manipulador.ANGULO_MAX);
    }

    // Retorna o ângulo atual
    public double getAngulo() {
        return anguloAtual;
    }

    // Retorna o ângulo desejado
    public double getSetpoint() {
        return setpoint;
    }

    // Verifica se chegou ao ângulo desejado
    public boolean estaNoSetpoint() {
        return Math.abs(setpoint - anguloAtual) <= Constants.Manipulador.TOLERANCIA;
    }

    // Verifica se o braço está no zero
    public boolean estaNoZero() {
        return Math.abs(anguloAtual) <= Constants.Manipulador.TOLERANCIA;
    }

    // Comando para movimentar o braço até um ângulo
    public Command irParaAngulo(double angulo) {
        return Commands.run(
                () -> definirAngulo(angulo),
                this).until(this::estaNoSetpoint);
    }

    // Comando: leva o braço até o ângulo máximo, arremessa a algae e volta ao
    // início.
    // Só executa se a algae estiver no manipulador.
    public Command arremessarAlgae(intakeSub intake) {
        return Commands.sequence(
                irParaAngulo(Constants.Manipulador.ANGULO_MAX),
                Commands.runOnce(() -> intake.definirPosicaoalgae(
                        intakeSub.posicaoalgae.ARREMESSADA)),
                Commands.waitSeconds(0.5),
                irParaAngulo(Constants.Manipulador.ANGULO_INICIAL))
                .onlyIf(intake::temAlgaeNoManipulador);
    }

    @Override
    public void periodic() {

        // Simula o movimento gradual do braço
        double diferenca = setpoint - anguloAtual;

        double variacaoMaxima = Constants.Manipulador.VELOCIDADE_MAX
                * Constants.Simulacao.PERIOD;

        double variacao = MathUtil.clamp(
                diferenca,
                -variacaoMaxima,
                variacaoMaxima);

        anguloAtual += variacao;

        // Atualiza a representação visual
        braco.setAngle(anguloAtual);

        // Publica os valores
        SmartDashboard.putNumber(
                "Manipulador/Angulo",
                anguloAtual);

        SmartDashboard.putNumber(
                "Manipulador/Setpoint",
                setpoint);

        SmartDashboard.putBoolean(
                "Manipulador/NoZero",
                estaNoZero());
    }
}