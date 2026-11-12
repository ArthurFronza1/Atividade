
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainSub;

public class NavegarParaPoseCMD extends Command {

    // Posição de destino
    private static final double X_DESTINO = 7.8;
    private static final double Y_DESTINO = 4.0;
    private static final double ANGULO_DESTINO = 60.0;

    // Tolerâncias
    private static final double TOLERANCIA_DISTANCIA = 0.08;
    private static final double TOLERANCIA_ANGULO = 2.0;

    // Controladores PID
    private final PIDController distanciaPID = new PIDController(0.7, 0.0, 0.0);

    private final PIDController giroPID = new PIDController(2.5, 0.0, 0.0);

    private final drivetrainSub drivetrain;

    private boolean chegouNaPosicao = false;

    public NavegarParaPoseCMD(drivetrainSub drivetrain) {

        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

        giroPID.enableContinuousInput(-Math.PI, Math.PI);
        giroPID.setTolerance(Math.toRadians(TOLERANCIA_ANGULO));

        distanciaPID.setTolerance(TOLERANCIA_DISTANCIA);
    }

    @Override
    public void initialize() {

        chegouNaPosicao = false;

        distanciaPID.reset();
        giroPID.reset();
    }

    @Override
    public void execute() {

        Pose2d poseAtual = drivetrain.getPose();

        double erroX = X_DESTINO - poseAtual.getX();
        double erroY = Y_DESTINO - poseAtual.getY();

        double distancia = Math.hypot(erroX, erroY);

        // Primeira etapa: deslocamento até a posição
        if (distancia > TOLERANCIA_DISTANCIA) {

            double anguloAlvo = Math.atan2(erroY, erroX);

            double velocidade = distanciaPID.calculate(
                    0.0,
                    distancia);

            double giro = giroPID.calculate(
                    poseAtual.getRotation().getRadians(),
                    anguloAlvo);

            velocidade = MathUtil.clamp(velocidade, 0.0, 0.5);
            giro = MathUtil.clamp(giro, -1.0, 1.0);

            drivetrain.drive(velocidade, giro);

        } else {

            // Segunda etapa: ajustar a orientação final
            chegouNaPosicao = true;

            double anguloFinal = Math.toRadians(ANGULO_DESTINO);

            double giro = giroPID.calculate(
                    poseAtual.getRotation().getRadians(),
                    anguloFinal);

            giro = MathUtil.clamp(giro, -1.0, 1.0);

            drivetrain.drive(0.0, giro);
        }
    }

    @Override
    public boolean isFinished() {

        return chegouNaPosicao && giroPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

        drivetrain.drive(0.0, 0.0);
    }
}
