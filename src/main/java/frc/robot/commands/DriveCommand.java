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
public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TabiSubsystem m_subsystem;
  private double m_initialDistance = 0.0;
  private double m_goalDistance = 0.0;

  PIDController drivePid = new PIDController(3.0, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(double goal, TabiSubsystem subsystem) {
    m_subsystem = subsystem;
    m_goalDistance = goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    drivePid.setTolerance(0.05);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialDistance = m_subsystem.getAverageDistance(); //Averge of both motor distances
    drivePid.reset();
    SmartDashboard.putNumber("initial distance", m_initialDistance);
    SmartDashboard.putNumber("target distance", m_initialDistance + m_goalDistance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double speed = drivePid.calculate(m_subsystem.getAverageDistance(), m_initialDistance + m_goalDistance);
      speed = MathUtil.clamp(speed, -0.25, 0.25);
      SmartDashboard.putNumber("distance", m_subsystem.getAverageDistance()); 
      SmartDashboard.putNumber("speed", speed);
      m_subsystem.arcadedrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePid.atSetpoint();
  }
}
