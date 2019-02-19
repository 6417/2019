/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SLiftingUnit extends Subsystem {

  private EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);
  
  private boolean zeroed = false, motionMagicEnabled = false;
  
  ShuffleboardLayout liftingunitSettings = Robot.shuffleSettings.getLayout("Lifting Unit", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,2);
  NetworkTableEntry maximumRaiseSpeed = liftingunitSettings.add("Lift Maximum Raise Speed", 0.5).getEntry();
  NetworkTableEntry maximumLoweringSpeed = liftingunitSettings.add("Lift Maximum Lowering Speed", -0.1).getEntry();
  NetworkTableEntry manualHoldOffset = liftingunitSettings.add("Lift Manual Hold Offset", 0.2).getEntry();

  /**
   * position in mm
   */
  private double targetPosition = 0;

  public SLiftingUnit() {
    super();
    addChild(Motors.liftMaster);
    addChild(Motors.liftFollower);
    
  }

  public double getMaximumRaiseSpeed() {
    return maximumRaiseSpeed.getDouble(0.5);
  }

  public void setMaximumRaiseSpeed(double maxRaise) {
    maximumRaiseSpeed.setDouble(maxRaise);
  }

  public double getMaximumLoweringSpeed() {
    return maximumLoweringSpeed.getDouble(-0.1);
  }

  public void setMaximumLoweringSpeed(double maxLowering) {
    maximumLoweringSpeed.setDouble(maxLowering);
  }

  public double getManualHoldOffset() {
    return manualHoldOffset.getDouble(0.2);
  }

  public void setManualHoldOffset(double manualHold) {
    manualHoldOffset.setDouble(manualHold);
  }

  /**
   * Returns position of the cart in mm.
   * 
   * @return Position in mm
   */
  public double getPosition() {
    return encoderConverter.getDistance(Motors.liftMaster.getSelectedSensorPosition());
  }

  /**
   * Sets the target position in mm
   */
  public void setTargetPosition(double position) {
    targetPosition = Algorithms.limit(position, 0, RobotMap.LIFTING_UNIT_DRIVE_LENGTH);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void drive(double value) {
    if(isMotionMagicEnabled()) {
      System.out.println("Target: " + getTargetPosition());
      Motors.liftMaster.set(ControlMode.MotionMagic, getTargetPosition(), DemandType.ArbitraryFeedForward, getManualHoldOffset());
      // Motors.liftMaster.set(mode, demand0, demand1Type, demand1);
    } else {
      // scale -1..1 to maximum lowering speed and maximum raise speed.
      value *= Math.abs(Algorithms.scale(value, -1, 1, getMaximumLoweringSpeed(), getMaximumRaiseSpeed()-getManualHoldOffset()));
      System.out.println("Scaled input: " + value + " Speed output: " + (value + getManualHoldOffset()));
      Motors.liftFollower.set(ControlMode.PercentOutput, value + getManualHoldOffset());
    }
  }

  public void drive() {
    drive(0);
  }

  public void setMotionMagicEnabled(boolean enabled) {
    // if (isZeroed()) {
      motionMagicEnabled = enabled;
    // }
  }

  public boolean isMotionMagicEnabled() {
    return motionMagicEnabled;
  }

  /**
   * Stops the motor for the cart.
   */
  public void stopMotor() {
    Motors.liftMaster.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Checking if the zero position is hit once. This method should be called often
   * in order to have a functioning Cart system.
   */
  public void checkZeroPosition() {
    if (!Motors.liftMaster.getSensorCollection().isRevLimitSwitchClosed()) {
      zeroed = true;
      Motors.liftMaster.setSelectedSensorPosition(0);
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
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.setActuator(true);
    // builder.setSafeState(this::stopMotor);
    // builder.addDoubleProperty("Motor Speed", Motors.liftMaster::getSelectedSensorVelocity, Motors.liftMaster::set);
    // builder.addBooleanProperty("Reverse Limit", Motors.liftMaster.getSensorCollection()::isRevLimitSwitchClosed, null);
    // builder.addBooleanProperty("Forward limit", Motors.liftMaster.getSensorCollection()::isFwdLimitSwitchClosed, null);
    // builder.addBooleanProperty("Zeroed", this::isZeroed, null);
    // builder.addDoubleProperty("Position (mm)", this::getPosition, null);
    builder.addDoubleProperty("Position raw (pulses)", Motors.liftMaster::getSelectedSensorPosition, null);
    // builder.addDoubleProperty("Target position (mm)", this::getTargetPosition, this::setTargetPosition);
    // builder.addDoubleProperty("Distance per Pulse", encoderConverter::getDistancePerPulse,
    //     encoderConverter::setDistancePerPulse);
    // builder.addBooleanProperty("Enable Motion Magic", this::isMotionMagicEnabled, this::setMotionMagicEnabled);
   }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
