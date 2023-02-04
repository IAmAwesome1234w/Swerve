// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANCoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  
  public SwerveModule(int driveMotorID, int turnMotorID,
   boolean driveMotorReversed,boolean turnMotorReversed,
    int absoluteEncoderID, double absoluteEncoderOffset,
     boolean absoluteEncoderReversed)
     {
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        absoluteEncoder = new CANCoder(absoluteEncoderID);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
     }

     public double getDriveMotorVelocity()
     {
      return driveEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
     }
    public double getTurnMotorPosition()
    {
      return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
    }
     public double getAbsoluteEncoderAngle()
     {
       double angle = absoluteEncoder.getAbsolutePosition();
       angle = angle - absoluteEncoderOffset;
       angle = angle*(Math.PI/180);
       return angle*(absoluteEncoderReversed ? -1.0 : 1.0);
     }

     public void resetEncoders()
     {
      driveEncoder.setPosition(0);
      turnEncoder.setPosition(getAbsoluteEncoderAngle()/SwerveConstants.TURN_MOTOR_PCONVERSION);
     }

     public SwerveModuleState getSwerveState()
     {
      return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
     }
     public void setState(SwerveModuleState state)
     {
        state = SwerveModuleState.optimize(state, getSwerveState().angle);
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
        turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), state.angle.getRadians()));
     }
    public void stop()
    {
      driveMotor.set(0);
      turnMotor.set(0);
    }
     

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
