// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.DriveToPoint;

public class TabiSubsystem extends SubsystemBase {
  SparkMax leftMotor = new SparkMax(DriveTrainConstants.kLeftMotorCanID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(DriveTrainConstants.kRightMotorCanID, MotorType.kBrushless);
  DifferentialDrive m_DifferentialDrive;

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  DifferentialDriveKinematics m_DriveKinematics;
  DifferentialDriveOdometry m_Odometry;
  DifferentialDrivePoseEstimator m_pPoseEstimator;

  Field2d m_Field = new Field2d();

  PIDController drivePid = new PIDController(2.0, 0, 0);
  PIDController leftDrivePid = new PIDController(2.0, 0, 0);
  PIDController rightDrivePid = new PIDController(2.0, 0, 0);
  PIDController turnPid = new PIDController(0.05, 0, 0);

  double targetMeters = 0.0;
  double rightTargetMeters = 0.0;
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
      config.encoder.positionConversionFactor(DriveTrainConstants.kMetersPerRotation);
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
      config.encoder.positionConversionFactor(DriveTrainConstants.kMetersPerRotation);
      //config.encoder.positionConversionFactor(ChassisConstants.kPositionConversionFactor);
      rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      m_DifferentialDrive = new DifferentialDrive(leftMotor, rightMotor);
      drivePid.setTolerance(0.1, 0.01);
      turnPid.setTolerance(2);
      m_Odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d(0,0, new Rotation2d()));
      m_DriveKinematics = new DifferentialDriveKinematics(0.29);
      m_pPoseEstimator = new DifferentialDrivePoseEstimator(m_DriveKinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d(0, 0, new Rotation2d(0)));

      SmartDashboard.putData("field", m_Field);

  }
  public void arcadeDrive(double speed, double turn){
    m_DifferentialDrive.arcadeDrive(speed, turn);
    SmartDashboard.putNumber("speed",speed);
    SmartDashboard.putNumber("turn",turn);
  }
  public void resetOdometry(){
    rightMotor.getEncoder().setPosition(0);
    leftMotor.getEncoder().setPosition(0);
    gyro.reset();
    m_pPoseEstimator.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d(0, 0, new Rotation2d()));
  }

  public double getAngle(){
    return gyro.getAngle();
  }

  public Pose2d getPose() {
      return m_pPoseEstimator.getEstimatedPosition();
  }

  public double getLeftDistance(){
    return leftMotor.getEncoder().getPosition();
  }
  public double getRightDistance(){
    return rightMotor.getEncoder().getPosition();
  }
  public double getAverageDistance(){
    return (getLeftDistance() + getRightDistance()) / 2;
  }


  public Command driveDistance(double distance){
    return startRun(()->{
      //leftMotor.getEncoder().setPosition(0);
      //rightMotor.getEncoder().setPosition(0);
      //getVelocity();
      double avgPos = leftMotor.getEncoder().getPosition() / 2 + rightMotor.getEncoder().getPosition()/2;
      targetMeters = avgPos + distance;

      drivePid.reset();
      SmartDashboard.putNumber("target", targetMeters);

      
    }, ()->{
      double avgPos = leftMotor.getEncoder().getPosition() / 2 + rightMotor.getEncoder().getPosition()/2;
      double speed = drivePid.calculate(avgPos, targetMeters);
      speed = MathUtil.clamp(speed, -0.25, 0.25);
      arcadeDrive(speed, 0);
      SmartDashboard.putNumber("position",avgPos);
      // SmartDashboard.putNumber("right position", rightMotor.getEncoder().getPosition());
    }).until(()->{
      return drivePid.atSetpoint();
      // leftMotor.getEncoder().getPosition()>distance;
    });
  }

  public Command turnDistance(double angle){
    return startRun(()->{
      gyro.reset();
    }, ()->{
      double rotation = -turnPid.calculate(gyro.getAngle(), angle);
      rotation = MathUtil.clamp(rotation, -0.25, 0.25);
      SmartDashboard.putNumber("orientation", gyro.getAngle()); //gyro.getYaw() last
      SmartDashboard.putNumber("rotation", rotation);
      arcadeDrive(0, rotation);
    }).until(()->{
      return turnPid.atSetpoint();
      // leftMotor.getEncoder().getPosition()>distance;
    });
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

  public Command runObstacleCourse() {
    Translation2d aB = new Translation2d(Meters.convertFrom(149, Inches), 0); //y
    Translation2d bC = aB.plus(new Translation2d(-Meters.convertFrom(17, Inches), 0)); //y
    Translation2d cD = bC.plus(new Translation2d(0, (Meters.convertFrom(106, Inches)))); //y
    Translation2d dE = cD.plus(new Translation2d(-Meters.convertFrom(74, Inches), 0)); //y
    Translation2d eF = dE.plus(new Translation2d(Meters.convertFrom(36, Inches), 0)); //y
    Translation2d fG = eF.plus(new Translation2d(Meters.convertFrom(12, Inches), Meters.convertFrom(70, Inches))); //y
    Translation2d gH = fG.plus(new Translation2d(0, Meters.convertFrom(34, Inches))); //y
    Translation2d hI = gH.plus(new Translation2d(-Meters.convertFrom(71, Inches), 0)); 
    Translation2d iJ = hI.plus(new Translation2d(-Meters.convertFrom(65, Inches), 0));
    Translation2d jK = iJ.plus(new Translation2d(0, -Meters.convertFrom(36, Inches)));

    //Translation2d pE = pD.plus(new Translation2d(-2.16, 0));
    return Commands.sequence(
      new DriveToPoint(new Pose2d(aB, new Rotation2d()), this), 
      new DriveToPoint(new Pose2d(bC, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(cD, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(dE, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(eF, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(fG, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(gH, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(hI, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(iJ, new Rotation2d()), this),
      new DriveToPoint(new Pose2d(jK, new Rotation2d()), this)

    );
  }

  @Override
  public void periodic() {
    Pose2d pose = m_pPoseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("pose/x", pose.getX());
    SmartDashboard.putNumber("pose/y", pose.getY());
    SmartDashboard.putNumber("pose/rotation", pose.getRotation().getDegrees());
    m_Field.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
