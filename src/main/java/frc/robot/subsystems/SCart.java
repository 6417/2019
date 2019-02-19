/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Susystem to move the tower across the robot
 */
public class SCart extends Subsystem {

  EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);
  boolean m_isHomed;
  /**
   * Limit switches of the cart are connected to a remote Talon SRX
   */
  FridolinsTalonSRX remoteTalon = new FridolinsTalonSRX(RobotMap.CART_REMOTE_LIMIT_SWITCH_ID);
  @Override
  protected void initDefaultCommand() {

  }

  public SCart() {
    setSubsystem("Cart");
    addChild(Motors.cartMotor);
    resetSubsystem();
  }

  private void resetSubsystem() {
    m_isHomed = false;
  }

  public boolean isHomed() {
    return m_isHomed;
  }
  /**
   * Returns position of the cart in mm.
   * @return Position in mm
   */
  public double getPosition() {
    return encoderConverter.getDistance(Motors.cartMotor.getSelectedSensorPosition());
  }

  /**
   * Moves the cart to the desired position.
   * @param targetPos Position of the cart in mm measured from the 0 point.
   */
  public void setPosition(double targetPos) {
    if(Robot.hatchGripper.isExtended()) {
      targetPos = encoderConverter.getPulses(Algorithms.limit(targetPos, 0, RobotMap.CART_DRIVE_LENGTH_HATCH_MM));
    } else {
      targetPos = encoderConverter.getPulses(Algorithms.limit(targetPos, 0, RobotMap.CART_DRIVE_LENGTH_MM));
    }

    // when the system is not homed, do not drive the cart!
    if(!isHomed()) {
      return;
    }

    // TODO check if position is allowed in regard of height of the lifting unit

    Motors.cartMotor.set(ControlMode.MotionMagic, targetPos);
  }

  public void driveManual(double speed) {
    if(Robot.hatchGripper.isExtended() && speed >= 0 && getPosition() >= RobotMap.CART_DRIVE_LENGTH_HATCH_MM - 100) {
      speed = 0;
      Motors.cartMotor.set(ControlMode.PercentOutput, speed);
    } else {
      Motors.cartMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  /**
   * Stops the motor for the cart.
   */
  public void stopMotor() {
    Motors.cartMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Check Limit switches and set the encoder to the specific encoder ticks
   */
  public void checkLimitSwitches() {
    if(!remoteTalon.getSensorCollection().isRevLimitSwitchClosed()) {
      Motors.cartMotor.setSelectedSensorPosition(0);
      m_isHomed = true;
    } else if(!remoteTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
      Motors.cartMotor.setSelectedSensorPosition(RobotMap.CART_DRIVE_LENGTH);
      m_isHomed = true;
    }
  }

  /**
   * Runs a automatic calibration routine to find it's zero point.
   * @return true when zero point found, false when not.
   */
  public boolean calibrate() {
    driveManual(-0.1);
    return !remoteTalon.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isInFrontWindow() {
    return (getPosition() >= RobotMap.CART_DRIVE_LENGTH - RobotMap.CART_WINDOW_LENGTH);
  }

  public boolean isInBackWindow() {
    return (getPosition() <= RobotMap.CART_WINDOW_LENGTH);
  }

  @Override
  public void periodic() {
    checkLimitSwitches();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", Motors.cartMotor::get, Motors.cartMotor::set);
    builder.addBooleanProperty("Reverse Limit", remoteTalon.getSensorCollection()::isRevLimitSwitchClosed, null);
    builder.addBooleanProperty("Forward limit", remoteTalon.getSensorCollection()::isFwdLimitSwitchClosed, null);
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addBooleanProperty("In Front Window", this::isInFrontWindow, null);
    builder.addBooleanProperty("In Back Window", this::isInBackWindow, null);
  }

}
