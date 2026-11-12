
package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutomacaoAlgaeCMD;
import frc.robot.commands.TransferirCMD;
import frc.robot.commands.NavegarParaPoseCMD;

import frc.robot.subsystems.drivetrainSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.elevadorSub;
import frc.robot.subsystems.manipuladorSub;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

        // Subsistema de exemplo
        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

        // Subsistemas do robô
        private final intakeSub intake = new intakeSub();
        private final elevadorSub elevador = new elevadorSub();
        private final manipuladorSub manipulador = new manipuladorSub();
        private final drivetrainSub drivetrain = new drivetrainSub();

        // Controle Xbox
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        public RobotContainer() {

                configureBindings();

                // Controle manual do movimento do robô
                drivetrain.setDefaultCommand(
                                Commands.run(
                                                () -> drivetrain.drive(
                                                                -m_driverController.getLeftY(),
                                                                -m_driverController.getRightX()),
                                                drivetrain));
        }

        private void configureBindings() {

                // Comando de exemplo
                new Trigger(m_exampleSubsystem::exampleCondition)
                                .onTrue(new ExampleCommand(m_exampleSubsystem));

                // A: coleta a Algae e transfere para o manipulador
                m_driverController.a().onTrue(
                                intake.coletarAlgae()
                                                .andThen(intake.transferirAlgae()));

                // B: arremessa a Algae
                m_driverController.b().onTrue(
                                manipulador.arremessarAlgae(intake));

                // X: abre/fecha o Intake manualmente
                m_driverController.x().onTrue(
                                intake.alternarIntake());

                // Y: zera o estado da Algae
                m_driverController.y().onTrue(
                                Commands.runOnce(
                                                () -> intake.definirPosicaoalgae(
                                                                intakeSub.posicaoalgae.FORA)));

                // LB: navegação automática até a posição definida
                m_driverController.leftBumper().onTrue(
                                new NavegarParaPoseCMD(drivetrain));
        }

        // Comando de automação da Algae
        public AutomacaoAlgaeCMD getAutomacaoAlgaeCommand() {

                return new AutomacaoAlgaeCMD(
                                elevador,
                                manipulador);
        }

        // Comando de transferência
        public Command getTransferirCommand() {

                return new TransferirCMD(
                                intake,
                                elevador,
                                manipulador);
        }

        public Command getAutonomousCommand() {

                return Autos.exampleAuto(m_exampleSubsystem);
        }
}
