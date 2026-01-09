// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;


public class TabiSubsystem extends SubsystemBase {
  SparkMax leftMotor = new SparkMax(DriveTrainConstants.kLeftMotorCanID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(DriveTrainConstants.kRightMotorCanID, MotorType.kBrushless);
  DifferentialDrive drivetrain;
  double targetMeters = 0;

  Optional<Trigger> lowGearTrigger;
  // Makes the trigger for the low gear function

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public Field2d field = new Field2d();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Meters.of(DriveTrainConstants.kTrackWidth));
  DifferentialDrivePoseEstimator odometry;
 
  

PIDController drivepid = new PIDController(2.0, 0, 0);
PIDController turnpid = new PIDController(0.05, 0, 0.005);
  public TabiSubsystem() {
    gyro.reset();
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      getLeftEncoder(),
      getRightEncoder(),
      new Pose2d(0, 0, new Rotation2d()));

    double wheelConversion = Inches.of(DriveTrainConstants.kWheelCircumference*DriveTrainConstants.kGearRatio).in(Meters);
    SmartDashboard.putNumber("drivetrain/wheelconversion", wheelConversion);
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.encoder.positionConversionFactor(wheelConversion).velocityConversionFactor(wheelConversion/60.0);

    leftConfig.inverted(false);
    leftConfig.openLoopRampRate(0.25);

    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.encoder.positionConversionFactor(wheelConversion).velocityConversionFactor(wheelConversion/60.0);
    rightConfig.inverted(true);
    rightConfig.openLoopRampRate(0.25);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    drivetrain = new DifferentialDrive(leftMotor, rightMotor);
    drivepid.setTolerance(0.1, 0.1);
    turnpid.setTolerance(2);
    SmartDashboard.putData(field);
  }
  public void reset() {
    gyro.reset();
    odometry.resetPose(new Pose2d(0, 0, new Rotation2d()));
  }

  public void setGearTrigger(Trigger t) {
    lowGearTrigger = Optional.of(t);
    // Sets t to a trigger and sending it to RobotContainer
  }

  public void arcadedrive(double speed, double turn){
    if (lowGearTrigger.isPresent()) {
      if (lowGearTrigger.get().getAsBoolean()) {
        speed /= 4;
        turn /= 4;
      }
    }
    drivetrain.arcadeDrive(speed, turn);
    // The mathematics for the high/low gear and ArcadeDrive
  }

public double getAngle() {
  return gyro.getAngle();
}
public double getLeftEncoder() {
  double encoder = leftMotor.getEncoder().getPosition();
  SmartDashboard.putNumber("bot/left", encoder);
  return encoder;
}
public double getRightEncoder() {
  double encoder = rightMotor.getEncoder().getPosition();
  SmartDashboard.putNumber("bot/right", encoder);
  return encoder;
}
public Pose2d getEstimatedPosition() {
  return odometry.getEstimatedPosition();
}

public double getAverageDistance(){
  return (getLeftEncoder() + getRightEncoder()) / 2;
}

public Command driveDistance(double distance){
  return startRun(()->{
    //leftMotor.getEncoder().setPosition(0);
    //rightMotor.getEncoder().setPosition(0);
    //getVelocity();
    double avgPos = leftMotor.getEncoder().getPosition() / 2 + rightMotor.getEncoder().getPosition()/2;
    targetMeters = avgPos + distance;

    drivepid.reset();
    SmartDashboard.putNumber("target", targetMeters);

    
  }, ()->{
    double avgPos = leftMotor.getEncoder().getPosition() / 2 + rightMotor.getEncoder().getPosition()/2;
    double speed = drivepid.calculate(avgPos, targetMeters);
    speed = MathUtil.clamp(speed, -0.25, 0.25);
    arcadedrive(speed, 0);
    SmartDashboard.putNumber("position",avgPos);
    // SmartDashboard.putNumber("right position", rightMotor.getEncoder().getPosition());
  }).until(()->{
    return drivepid.atSetpoint();
    // leftMotor.getEncoder().getPosition()>distance;
  });
}

public Command turnDistance(double angle){
  return startRun(()->{
    gyro.reset();
  }, ()->{
    double rotation = -turnpid.calculate(gyro.getAngle(), angle);
    rotation = MathUtil.clamp(rotation, -0.25, 0.25);
    SmartDashboard.putNumber("orientation", gyro.getAngle()); //gyro.getYaw() last
    SmartDashboard.putNumber("rotation", rotation);
    arcadedrive(0, rotation);
  }).until(()->{
    return turnpid.atSetpoint();
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

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), new DifferentialDriveWheelPositions(getLeftEncoder(), getRightEncoder()));
    Pose2d bot = odometry.getEstimatedPosition();
    field.setRobotPose(bot);
    SmartDashboard.putNumber("pose/X", bot.getX());
    SmartDashboard.putNumber("pose/Y", bot.getY());
    SmartDashboard.putNumber("pose/rotation", bot.getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
