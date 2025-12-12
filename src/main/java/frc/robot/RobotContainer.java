// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.TabiSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;//CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TabiSubsystem m_exampleSubsystem = new TabiSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID m_driverController = new CommandGenericHID(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new TurnCommand(45, m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.button(OperatorConstants.kDriverControllerA).whileTrue(m_exampleSubsystem.run(
        () -> {
          m_exampleSubsystem.arcadeDrive(0.15, 0);
        }

    ).withTimeout(2.0));
    m_driverController.pov(OperatorConstants.kDriverControllerPOVRight).onTrue(new TurnCommand(45, m_exampleSubsystem));
    m_driverController.pov(OperatorConstants.kDriverControllerPOVLeft).onTrue(new TurnCommand(-45, m_exampleSubsystem));
    m_driverController.pov(OperatorConstants.kDriverControllerPOVUp).onTrue(new DriveCommand(0.5, m_exampleSubsystem));
    m_driverController.pov(OperatorConstants.kDriverControllerPOVDown).onTrue(new DriveCommand(-0.5, m_exampleSubsystem));
    // m_driverController.button(OperatorConstants.kDriverControllerB).onTrue(m_exampleSubsystem.driveDistance(2));
    m_driverController.button(OperatorConstants.kDriverControllerB).onTrue(m_exampleSubsystem.runObstacleCourse());
    m_driverController.button(OperatorConstants.kDriverControllerY).onTrue(new DriveToPoint(new Pose2d(1,0.5, new Rotation2d()), m_exampleSubsystem));
    m_driverController.button(OperatorConstants.kDriverControllerX).onTrue(new DriveToPoint(new Pose2d(-1,-0.5, new Rotation2d()), m_exampleSubsystem));
    // m_driverController.button(OperatorConstants.kDriverControllerA).onTrue(
    //   Commands.sequence(
    //     new DriveToPoint(new Pose2d(1, 0, new Rotation2d()), m_exampleSubsystem),
    //     new DriveToPoint(new Pose2d(1, 1, new Rotation2d()), m_exampleSubsystem),
    //     new DriveToPoint(new Pose2d(0, 1, new Rotation2d()), m_exampleSubsystem),
    //     new DriveToPoint(new Pose2d(0, 0, new Rotation2d()), m_exampleSubsystem)
    //   )  
    // );
    m_exampleSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              double gamepadDrive = m_driverController.getRawAxis(OperatorConstants.kControllerLeftVertical);
              double gamepadTurn = m_driverController.getRawAxis(OperatorConstants.kControllerLeftHorizontal);
              SmartDashboard.putNumber("gamepadDrive", gamepadDrive);
              SmartDashboard.putNumber("gamepadTurn", gamepadTurn);
              m_exampleSubsystem.arcadeDrive(-gamepadDrive, -gamepadTurn);
            }, m_exampleSubsystem));
  }
  public void resetOdometry(){
    m_exampleSubsystem.resetOdometry();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
