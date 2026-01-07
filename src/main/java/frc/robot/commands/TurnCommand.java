// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TabiSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TabiSubsystem m_subsystem;
  private double m_initialAngle = 0.0;
  private double m_goalAngle = 0.0;

  PIDController turnPid = new PIDController(0.05, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnCommand(double goal, TabiSubsystem subsystem) {
    m_subsystem = subsystem;
    m_goalAngle = goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    turnPid.setTolerance(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialAngle = m_subsystem.getAngle();
    turnPid.reset();
    SmartDashboard.putNumber("initial angle", m_initialAngle);
    SmartDashboard.putNumber("target angle", m_initialAngle + m_goalAngle);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double rotation = turnPid.calculate(m_subsystem.getAngle(), m_initialAngle + m_goalAngle);
      rotation = MathUtil.clamp(rotation, -0.25, 0.25);
      SmartDashboard.putNumber("orientation", m_subsystem.getAngle()); //gyro.getYaw() last
      SmartDashboard.putNumber("rotation", rotation);
      m_subsystem.arcadedrive(0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPid.atSetpoint();
  }
}
