package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrainSub extends SubsystemBase {

    // Limites da simulação
    private static final double VELOCIDADE_MAX = 2.0; // m/s
    private static final double VELOCIDADE_GIRO_MAX = 3.0; // rad/s (usado no autônomo / navegação)
    private static final double VELOCIDADE_GIRO_TELEOP = 0.3; // rad/s (seu valor original, só para o controle)

    private static final double PERIOD = 0.02;
    private static final double DEADBAND = 0.08;

    // Desafio 2: pose de destino (lado azul)
    public static final Pose2d POSE_DESTINO = new Pose2d(7.8, 4.0, Rotation2d.fromDegrees(60));

    private static final PathConstraints LIMITES = new PathConstraints(
            1.8, 2.0,
            Units.degreesToRadians(180),
            Units.degreesToRadians(360));

    // Pose inicial (o autônomo redefine para 7,2 / 1,0 via PathPlanner)
    private Pose2d pose = new Pose2d(2.0, 2.0, new Rotation2d());

    // Velocidades atuais já em unidades reais
    private double velocidadeFrente = 0.0; // m/s
    private double velocidadeGiro = 0.0; // rad/s

    private final Field2d field = new Field2d();

    public drivetrainSub() {
        field.setRobotPose(pose);
        SmartDashboard.putData("Drivetrain/Field", field);

        // Mostra o trajeto e o alvo do PathPlanner no campo
        PathPlannerLogging.setLogActivePathCallback(
                poses -> field.getObject("path").setPoses(poses));
        PathPlannerLogging.setLogTargetPoseCallback(
                alvo -> field.getObject("alvo").setPose(alvo));

        configurarAutoBuilder();
    }

    private void configurarAutoBuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPLTVController(PERIOD), // robô não-holonômico (frente + giro)
                config,
                () -> false, // enunciado usa coordenadas do lado azul; não espelhar
                this);
    }

    // ---------- Controle manual (Xbox) ----------

    public void drive(double frente, double giro) {
        velocidadeFrente = MathUtil.applyDeadband(frente, DEADBAND) * VELOCIDADE_MAX;
        velocidadeGiro = MathUtil.applyDeadband(giro, DEADBAND) * VELOCIDADE_GIRO_TELEOP;
    }

    // ---------- Interface exigida pelo PathPlanner ----------

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d novaPose) {
        pose = novaPose;
        field.setRobotPose(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds(velocidadeFrente, 0.0, velocidadeGiro);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        velocidadeFrente = MathUtil.clamp(
                speeds.vxMetersPerSecond, -VELOCIDADE_MAX, VELOCIDADE_MAX);
        velocidadeGiro = MathUtil.clamp(
                speeds.omegaRadiansPerSecond, -VELOCIDADE_GIRO_MAX, VELOCIDADE_GIRO_MAX);
    }

    // ---------- Desafio 2: navegação até uma pose ----------

    /**
     * Vai até a pose (de qualquer posição/orientação inicial) e, no final,
     * gira no lugar até o ângulo desejado. Num robô não-holonômico o PathPlanner
     * não controla o ângulo final, por isso o giro extra.
     */
    public Command irParaPose(Pose2d destino) {
        return AutoBuilder.pathfindToPose(destino, LIMITES, 0.0)
                .andThen(girarParaAngulo(destino.getRotation()));
    }

    public Command girarParaAngulo(Rotation2d alvo) {
        PIDController pid = new PIDController(4.0, 0.0, 0.0);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.setTolerance(Math.toRadians(1.5));

        return run(() -> driveRobotRelative(new ChassisSpeeds(
                0.0,
                0.0,
                pid.calculate(pose.getRotation().getRadians(), alvo.getRadians()))))
                .beforeStarting(pid::reset)
                .until(pid::atSetpoint)
                .finallyDo(interrompido -> driveRobotRelative(new ChassisSpeeds()));
    }

    // ---------- Simulação ----------

    @Override
    public void periodic() {
        pose = pose.exp(new Twist2d(
                velocidadeFrente * PERIOD,
                0.0,
                velocidadeGiro * PERIOD));

        field.setRobotPose(pose);

        SmartDashboard.putNumber("Drivetrain/X", pose.getX());
        SmartDashboard.putNumber("Drivetrain/Y", pose.getY());
        SmartDashboard.putNumber("Drivetrain/Angulo", pose.getRotation().getDegrees());
    }
}