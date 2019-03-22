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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
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

  public static boolean lifting_unit_drive_permitted = false;

  private EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);
  
  private boolean zeroed = false;

  private boolean m_autonomous = false;

  private int m_minimumHeight = 0, m_maximumHeight = RobotMap.LIFTING_UNIT_DRIVE_LENGTH;

  ShuffleboardLayout liftingunitSettings = Robot.shuffleSettings.getLayout("Lifting Unit", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,2);
  NetworkTableEntry maximumRaiseSpeed = liftingunitSettings.add("Lift Maximum Raise Speed", 0.5).getEntry();
  NetworkTableEntry maximumLoweringSpeed = liftingunitSettings.add("Lift Maximum Lowering Speed", -0.1).getEntry();
  NetworkTableEntry manualHoldOffset = liftingunitSettings.add("Lift Manual Hold Offset", 0.1).getEntry();

  private int m_targetPosition = 0;

  public SLiftingUnit() {
    super();
    addChild(Motors.liftMaster);
    addChild(Motors.liftFollower);
    InstantCommand setForwardEncoder = new InstantCommand(new Runnable() {
      @Override
      public void run() {
        zeroed = true;
        Motors.liftFollower.setSelectedSensorPosition(-RobotMap.LIFTING_UNIT_DRIVE_LENGTH);
      }
    });
    setForwardEncoder.setRunWhenDisabled(true);
    forwardLimit.whenPressed(setForwardEncoder);

    InstantCommand setReverseEncoder = new InstantCommand(new Runnable() {
      @Override
      public void run() {
        zeroed = true;
        Motors.liftFollower.setSelectedSensorPosition(-RobotMap.LIFTING_UNIT_ZERO_POSITION);
      }
    });
    setReverseEncoder.setRunWhenDisabled(true);
    reverseLimit.whenPressed(setReverseEncoder);
  };

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
    return manualHoldOffset.getDouble(0.1);
  }

  public void setManualHoldOffset(double manualHold) {
    manualHoldOffset.setDouble(manualHold);
  }

  public int getMinimumHeight() {
    return this.m_minimumHeight;
  }

  public void setMinimumHeight(int minimumHeight) {
    this.m_minimumHeight = minimumHeight;
  }

  public int getMaximumHeight() {
    return this.m_maximumHeight;
  }

  public void setMaximumHeight(int maximumHeight) {
    this.m_maximumHeight = maximumHeight;
  }

  /**
   * Returns position of the cart in mm.
   * 
   * @return Position in mm
   */
  public double getPosition() {
    return Motors.liftMaster.getSelectedSensorPosition();
    // return encoderConverter.getDistance(Motors.liftMaster.getSelectedSensorPosition());
  }

  /**
   * Checks if lifting unit's position is in the desired range
   * @param position position to reach
   * @return true, when current position is in the range, false when not in range
   */
  public boolean isInRange(int position) {
    return (Math.abs(getPosition() - position) < RobotMap.LIFTING_UNIT_SAFETY_ZONE);
  }

  /**
   * Sets the target position in encoder ticks
   */
  public void setTargetPosition(int position) {
    m_targetPosition = Algorithms.limit(position, 0, RobotMap.LIFTING_UNIT_DRIVE_LENGTH);
  }

  public int getTargetPosition() {
    return m_targetPosition;
  }

  public void drive(double value) {
      if(m_autonomous) {
        driveAutonomous();
      } else {
        driveManual(value);
      }
  }

  public void drive() {
    drive(0);
  }
 
  public void driveAutonomous() {
    if(!zeroed) {
      stopMotor();
      return;
    }

    int targetPosition = Math.min(Math.max(getTargetPosition(), getMinimumHeight()), getMaximumHeight());

    Motors.liftMaster.set(ControlMode.MotionMagic, targetPosition);
  }

  public void driveManual(double value) {
    value *= Math.abs(Algorithms.scale(value, -1, 1, getMaximumLoweringSpeed(), getMaximumRaiseSpeed()-getManualHoldOffset()));
    Motors.liftMaster.set(ControlMode.PercentOutput, value + getManualHoldOffset());
  }

  public void enableAutonomous(boolean enable) {
    m_autonomous = enable;
  }

  public boolean isAutonomousEnabled() {
    return m_autonomous;
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

  static Button forwardLimit = new Button() {

    @Override
    public boolean get() {
      return !Motors.liftMaster.getSensorCollection().isFwdLimitSwitchClosed();
    }
  };

  static Button reverseLimit = new Button() {

    @Override
    public boolean get() {
      return !Motors.liftMaster.getSensorCollection().isRevLimitSwitchClosed();
    }
  };

  /**
   * Returns if the Subsystem has been zeroed at least once.
   * 
   * @return true when zeroed, false when not.
   */
  public boolean isZeroed() {
    return zeroed;
  }

  public void setZeroed(boolean zeroed) {
    this.zeroed = zeroed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.setActuator(true);
    // builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", Motors.liftMaster::getMotorOutputPercent, Motors.liftMaster::set);
    builder.addBooleanProperty("Reverse Limit", Motors.liftMaster.getSensorCollection()::isRevLimitSwitchClosed, null);
    builder.addBooleanProperty("Forward limit", Motors.liftMaster.getSensorCollection()::isFwdLimitSwitchClosed, null);
    builder.addBooleanProperty("Zeroed", this::isZeroed, null);
    builder.addDoubleProperty("Position (mm)", this::getPosition, null);
    builder.addDoubleProperty("Position raw (pulses)", Motors.liftMaster::getSelectedSensorPosition, null);
    // builder.addDoubleProperty("Target position (mm)", this::getTargetPosition, this::setTargetPosition);
    builder.addDoubleProperty("Distance per Pulse", encoderConverter::getDistancePerPulse,
        encoderConverter::setDistancePerPulse);
    builder.addBooleanProperty("Enable Motion Magic", this::isAutonomousEnabled, this::enableAutonomous);
   }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
