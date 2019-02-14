/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motors;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Susystem to move the tower across the robot
 */
public class SCart extends Subsystem {

  EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);

  @Override
  protected void initDefaultCommand() {

  }

  public SCart() {
    setSubsystem("Cart");
    addChild(Motors.cartMotor);
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
    targetPos = Algorithms.limit(targetPos, 0, RobotMap.CART_DRIVE_LENGTH);
    
    if (isInFrontWindow() && targetPos > RobotMap.CART_DRIVE_LENGTH) {
      Motors.cartMotor.set(ControlMode.MotionMagic, RobotMap.CART_DRIVE_LENGTH);
    } else if (isInBackWindow() && targetPos < 0) {
      Motors.cartMotor.set(ControlMode.MotionMagic, 0);
    } else {
      Motors.cartMotor.set(ControlMode.MotionMagic, targetPos);
    }

    SmartDashboard.putNumber("SensorVel", Motors.cartMotor.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("SensorPos", Motors.cartMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("MotorOutputPercent", Motors.cartMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("ClosedLoopError", Motors.cartMotor.getClosedLoopError(0));
    SmartDashboard.putNumber("Target Position", 0);
  }

  public void driveManual() {
    Motors.cartMotor.set(ControlMode.PercentOutput, -OI.JoystickMainDriver.getY());
  }

  /**
   * Stops the motor for the cart.
   */
  public void stopMotor() {
    Motors.cartMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Runs a manual calibration routine to find it's zero point.
   * @return true when zero point found, false when not.
   */
  public boolean calibrate() {
    driveManual();
    return !Motors.cartMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isInFrontWindow() {
    return (getPosition() >= RobotMap.CART_DRIVE_LENGTH - RobotMap.CART_WINDOW_LENGTH);
  }

  public boolean isInBackWindow() {
    return (getPosition() <= RobotMap.CART_WINDOW_LENGTH);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", Motors.cartMotor::get, Motors.cartMotor::set);
    builder.addBooleanProperty("Reverse Limit", Motors.cartMotor.getSensorCollection()::isRevLimitSwitchClosed, null);
    builder.addBooleanProperty("Forward limit", Motors.cartMotor.getSensorCollection()::isFwdLimitSwitchClosed, null);
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addBooleanProperty("In Front Window", this::isInFrontWindow, null);
    builder.addBooleanProperty("In Back Window", this::isInBackWindow, null);
  }

}
