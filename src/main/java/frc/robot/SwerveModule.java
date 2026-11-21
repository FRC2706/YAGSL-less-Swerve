// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  private static final double kDriveRatio = 6.75;
  private static final double kTurnRatio = 21.4286;

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final CANcoder absoluteEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SparkClosedLoopController m_drivePIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SparkClosedLoopController m_turningPIDController;

  /**
   * Constructs a SwerveModule with a drive motor controller, turning motor controller, and absolute encoder*/

  public SwerveModule(int driveCanID, int turnCanID, int absoluteEncoderCanID, double absoluteEncoderOffset, boolean invertAbsoluteEncoder, boolean invertRelativeEncoder) {
    //Construct motors
    m_driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turnCanID, MotorType.kBrushless);

    //Construct encoders
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    absoluteEncoder = new CANcoder(absoluteEncoderCanID);
    
    //Construct PID controllers
    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turningPIDController = m_turningMotor.getClosedLoopController();

    //Set up drive configs
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    if (invertRelativeEncoder){
      driveConfig.inverted(true);
    }
    else{
      driveConfig.inverted(false);
    }
    driveConfig.idleMode(IdleMode.kCoast);
    driveConfig.closedLoop.p(0.0001);
    driveConfig.closedLoop.i(0);
    driveConfig.closedLoop.d(0.1);
    driveConfig.closedLoop.feedForward.kV(0);
    driveConfig.closedLoop.outputRange(-1, 1);
    driveConfig.closedLoopRampRate(0.25);
    driveConfig.smartCurrentLimit(40);
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Set up turn configs
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.inverted(true);
    turnConfig.closedLoop.p(0.025);
    turnConfig.closedLoop.i(0);
    turnConfig.closedLoop.d(0);
    turnConfig.closedLoop.feedForward.kV(0);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingMinInput(-0.5 * kTurnRatio);
    turnConfig.closedLoop.positionWrappingMaxInput(0.5 * kTurnRatio);
    turnConfig.closedLoop.outputRange(-1, 1);
    turnConfig.closedLoopRampRate(0.25);
    turnConfig.smartCurrentLimit(30);
    m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Set up absolute encoder offset
    CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; 
    if (invertAbsoluteEncoder){
      absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
    }
    else{
      absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; 
    }
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = -absoluteEncoderOffset; //Offset should be in rotations; It is the value when the bevel is pointed to the left when an offset of 0
    absoluteEncoderConfig.FutureProofConfigs = true;
    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);

    //Match up relative encoder with absolute encoder
    updateEncoder();
  }

  //Updates relative encoder values with absolute encoder values
  public void updateEncoder(){
    m_turningEncoder.setPosition(absoluteEncoder.getAbsolutePosition(true).getValueAsDouble() * kTurnRatio);
  }

  //Returns the angle of the module in degrees
  public Rotation2d getAngle(){
    double angle = (m_turningEncoder.getPosition() / kTurnRatio % 1)* 2 * Math.PI; //Get the angle value of the robot between 0 and 360
    //Normalize the angle between 0 - 360 degrees
    if (angle >= Math.PI){
      angle = angle - 2 * Math.PI;
    }
    else if (angle <= -Math.PI){
      angle = angle + 2 * Math.PI;
    }
    return new Rotation2d(angle);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity() / 60 * 2 * Math.PI * kWheelRadius / kDriveRatio, getAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition() * 2 * Math.PI * kWheelRadius, getAngle());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    var encoderRotation = getAngle();

    //Normalize the rotation to be a principal angle in case the wrapping doesn't work properly
    /** 
    if (m_turningEncoder.getPosition() < -0.5){
      encoderRotation = new Rotation2d(-(Math.abs(m_turningEncoder.getPosition() /kTurnRatio * 360) % 360));
    }
    else if (m_turningEncoder.getPosition() > 0.5){
      encoderRotation = new Rotation2d(Math.abs(m_turningEncoder.getPosition() /kTurnRatio * 360) % 360);
    }*/
    

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving. 
    desiredState.cosineScale(encoderRotation);

    //Set speed for drive motor
    m_drivePIDController.setSetpoint(desiredState.speedMetersPerSecond * 60 / (2 * Math.PI * kWheelRadius) * kDriveRatio, SparkBase.ControlType.kVelocity);

    //Set position for turning motor
    m_turningPIDController.setSetpoint(desiredState.angle.getRotations() * kTurnRatio, ControlType.kPosition);
  }
}
