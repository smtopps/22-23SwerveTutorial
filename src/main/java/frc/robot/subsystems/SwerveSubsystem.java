// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final double maxVoltage = 12.0;

  public final double maxVelocityMetersPerSecond = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond / Math.hypot(SwerveConstants.TRACKWIDTH_METERS / 2.0, SwerveConstants.WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(SwerveConstants.TRACKWIDTH_METERS / 2.0, SwerveConstants.WHEELBASE_METERS / 2.0), // Front Left
    new Translation2d(SwerveConstants.TRACKWIDTH_METERS / 2.0, -SwerveConstants.WHEELBASE_METERS / 2.0), // Front Right
    new Translation2d(-SwerveConstants.TRACKWIDTH_METERS / 2.0, SwerveConstants.WHEELBASE_METERS / 2.0), // Back Left
    new Translation2d(-SwerveConstants.TRACKWIDTH_METERS / 2.0, -SwerveConstants.WHEELBASE_METERS / 2.0) // Back Right
  );

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    ShuffleboardTab swerveTab = Shuffleboard.getTab("Serve Drive");

    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR,
      SwerveConstants.FRONT_LEFT_STEER_MOTOR,
      SwerveConstants.FRONT_LEFT_STEER_ENCODER,
      SwerveConstants.FRONT_LEFT_STEER_OFFSET);

    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2, 
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET);

    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2, 
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET);

    backRightModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2, 
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR,
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSecond);
    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[3].angle.getRadians());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }
  
  public void stop() {
    frontLeftModule.set(0, 0);
    frontRightModule.set(0, 0);
    backLeftModule.set(0, 0);
    backRightModule.set(0, 0);
  }
}
