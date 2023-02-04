// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenixpro.sim.ChassisReference;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private SwerveModule left_front = new SwerveModule(
  SwerveConstants.LEFT_FRONT_DRIVE_ID,
  SwerveConstants.LEFT_FRONT_TURN_ID,
  false,
  true,
  SwerveConstants.LEFT_FRONT_CANCODER_ID,
  SwerveConstants.LEFT_FRONT_OFFSET,
  false);

  private SwerveModule right_front = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID,
    SwerveConstants.RIGHT_FRONT_TURN_ID,
  false,
   true,
    SwerveConstants.RIGHT_FRONT_CANCODER_ID,
     SwerveConstants.RIGHT_FRONT_OFFSET,
     false);

  private SwerveModule left_back = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID,
     SwerveConstants.LEFT_BACK_TURN_ID,
      false,
       true,
        SwerveConstants.LEFT_BACK_CANCODER_ID,
         SwerveConstants.LEFT_BACK_OFFSET,
          false);

  private SwerveModule right_back =  new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID,
     SwerveConstants.RIGHT_BACK_TURN_ID,
      false,
       true, 
       SwerveConstants.RIGHT_BACK_CANCODER_ID, 
       SwerveConstants.RIGHT_BACK_OFFSET,
        false);
  
  private Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
  private SlewRateLimiter front_limiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter side_limiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turn_limiter =  new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);
  public Drivetrain() {
    
  }
  public void zeroHeading()
  {
    gyro.setYaw(0);
  }
  public void setHeading(Rotation2d angle)
  {
    gyro.setYaw(angle.getDegrees());
  }
  public double getHeading()
  {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }
  public Rotation2d getHeadingRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading());
  }
  public void swerveDrive(double frontspeed, double sidespeed, double turnspeed, boolean fieldOriented, boolean deadband)
  {
    if(deadband)
    {
      frontspeed = Math.abs(frontspeed) > 1.0 ? frontspeed : 0;
      turnspeed = Math.abs(turnspeed) > 1.0 ? turnspeed: 0;
      sidespeed = Math.abs(turnspeed) > 1.0 ? sidespeed: 0;
    }
    frontspeed = front_limiter.calculate(frontspeed) * SwerveConstants.TELE_DRIVE_MAX_ACCELERATION;
    sidespeed = side_limiter.calculate(sidespeed) * SwerveConstants.TELE_DRIVE_MAX_ACCELERATION;
    turnspeed = turn_limiter.calculate(turnspeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION;
    ChassisSpeeds chassisSpeeds;
    if(fieldOriented)
    {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontspeed, sidespeed, turnspeed, getHeadingRotation2d());
    }else{
      chassisSpeeds = new ChassisSpeeds(frontspeed, sidespeed, turnspeed);
    }
    SwerveModuleState moduleStates[] = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }
  public void setModuleStates(SwerveModuleState moduleStates[])
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    left_front.setState(moduleStates[0]);
    right_front.setState(moduleStates[1]);
    left_back.setState(moduleStates[2]);
    right_back.setState(moduleStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
