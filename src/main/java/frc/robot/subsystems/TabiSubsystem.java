// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class TabiSubsystem extends SubsystemBase {
  SparkMax leftMotor = new SparkMax(DriveTrainConstants.kLeftMotorCanID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(DriveTrainConstants.kRightMotorCanID, MotorType.kBrushless);
  DifferentialDrive m_DifferentialDrive;
  /** Creates a new ExampleSubsystem. */
  public TabiSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(80)
          .inverted(true);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.openLoopRampRate(0.25);
      //config.encoder.positionConversionFactor(ChassisConstants.kPositionConversionFactor);
      leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(80);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.inverted(false);
      config.openLoopRampRate(0.25);
      //config.encoder.positionConversionFactor(ChassisConstants.kPositionConversionFactor);
      rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_DifferentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }
  public void arcadeDrive(double speed, double turn){
    m_DifferentialDrive.arcadeDrive(speed, turn);
    SmartDashboard.putNumber("speed",speed);
    SmartDashboard.putNumber("turn",turn);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
