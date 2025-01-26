// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.Configs;

public class Module {
  private final SparkMax drivingSpark;
  private final SparkMax turningSpark;

  private final RelativeEncoder drivingEncoder;
  private final AnalogEncoder turningEncoder;

  private final SparkClosedLoopController drivingClosedLoopController;
  private final SparkClosedLoopController turningClosedLoopController;

  private double CAO = 0;
  private SwerveModuleState DS = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Thrifty Absolute
   * Encoder.
   */
  public Module(int drivingCANId, int turningCANId, int turningAnalogInputId, double chassisAngularOffset) {
    drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    drivingEncoder = drivingSpark.getEncoder();
    turningEncoder = new AnalogEncoder(turningAnalogInputId);

    drivingClosedLoopController = drivingSpark.getClosedLoopController();
    turningClosedLoopController = turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    drivingSpark.configure(Configs.MK4iSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turningSpark.configure(Configs.MK4iSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    CAO = chassisAngularOffset;
    DS.angle = new Rotation2d(turningEncoder.get());
    drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.get() - CAO));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.get() - CAO));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(CAO));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(turningEncoder.get()));

    // Command driving and turning SPARKS towards their respective setpoints.
    drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    DS = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  public double getAngle() {
    return turningEncoder.get();
  }
}