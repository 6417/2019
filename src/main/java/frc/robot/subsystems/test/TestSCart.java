/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Motors;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class TestSCart extends Subsystem {

  private EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);
  private IFridolinsMotors remoteLimitSwitch = new FridolinsTalonSRX(RobotMap.CART_REMOTE_LIMIT_SWITCH_ID);
  
  private boolean zeroed = false, motionMagicEnabled = false;
  /**
   * position in mm
   */
  private double targetPosition = 0;

  public TestSCart() {
    super();
    addChild(Motors.cartMotor);
  }

  /**
   * Returns position of the cart in mm.
   * 
   * @return Position in mm
   */
  public double getPosition() {
    return encoderConverter.getDistance(Motors.cartMotor.getSelectedSensorPosition());
  }

  /**
   * Sets the target position in mm
   */
  public void setTargetPosition(double position) {
    targetPosition = Algorithms.limit(position, 0, RobotMap.CART_DRIVE_LENGTH);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void drive(double value) {
    if(isMotionMagicEnabled()) {
      Motors.cartMotor.set(ControlMode.MotionMagic, getTargetPosition());
    }
    Motors.cartMotor.set(ControlMode.PercentOutput, value);
  }

  public void drive() {
    drive(0);
  }

  private void setMotionMagicEnabled(boolean enabled) {
    if (isZeroed()) {
      motionMagicEnabled = enabled;
    }
  }

  public boolean isMotionMagicEnabled() {
    return motionMagicEnabled;
  }

  /**
   * Stops the motor for the cart.
   */
  public void stopMotor() {
    Motors.cartMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Checking if the zero position is hit once. This method should be called often
   * in order to have a functioning Cart system.
   */
  public void checkZeroPosition() {
    if (!this.remoteLimitSwitch.isReverseLimitSwitchActive()) {
      zeroed = true;
      Motors.cartMotor.setSelectedSensorPosition(0);
    }
  }

  /**
   * Returns if the Subsystem has been zeroed at least once.
   * 
   * @return true when zeroed, false when not.
   */
  public boolean isZeroed() {
    return zeroed;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.setActuator(true);
    // builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", Motors.cartMotor::getSelectedSensorVelocity, Motors.cartMotor::set);
    builder.addBooleanProperty("Reverse Limit", this.remoteLimitSwitch::isReverseLimitSwitchActive, null);
    builder.addBooleanProperty("Forward limit", this.remoteLimitSwitch::isForwardLimitSwitchActive, null);
    builder.addBooleanProperty("Zeroed", this::isZeroed, null);
    builder.addDoubleProperty("Position (mm)", this::getPosition, null);
    builder.addDoubleProperty("Position raw (pulses)", Motors.cartMotor::getSelectedSensorPosition, null);
    builder.addDoubleProperty("Target position (mm)", this::getTargetPosition, this::setTargetPosition);
    builder.addDoubleProperty("Distance per Pulse", encoderConverter::getDistancePerPulse,
        encoderConverter::setDistancePerPulse);
    builder.addBooleanProperty("Enable Motion Magic", this::isMotionMagicEnabled, this::setMotionMagicEnabled);
   }
}