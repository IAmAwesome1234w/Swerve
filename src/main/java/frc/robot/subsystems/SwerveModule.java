// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SwerveModule leftfront_drive = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID,
    SwerveConstants.LEFT_FRONT_CANCODER_ID,
    SwerveConstants.
  );

  
  public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
