/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.TreeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;

import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import ch.fridolinsrobotik.utilities.EPositions;
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

  public static boolean lifting_unit_drive_permitted = false;

  private EncoderConverter encoderConverter = new EncoderConverter(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);
  
  private boolean zeroed = false, motionMagicEnabled = false;

  private boolean m_autonomous = false;

  private static Integer[] m_liftingUnitPositions;
  private int m_liftingUnitPosition = 0;

  ShuffleboardLayout liftingunitSettings = Robot.shuffleSettings.getLayout("Lifting Unit", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,2);
  NetworkTableEntry maximumRaiseSpeed = liftingunitSettings.add("Lift Maximum Raise Speed", 0.5).getEntry();
  NetworkTableEntry maximumLoweringSpeed = liftingunitSettings.add("Lift Maximum Lowering Speed", -0.1).getEntry();
  NetworkTableEntry manualHoldOffset = liftingunitSettings.add("Lift Manual Hold Offset", 0.1).getEntry();

  /**
   * position in mm
   */
  private double m_practicalTargetPosition = 0;

  private double m_theoreticalTargetPosition = 0;

  public SLiftingUnit() {
    super();
    addChild(Motors.liftMaster);
    addChild(Motors.liftFollower);

    TreeSet<Integer> m_liftingUnitTreeSet = new TreeSet<Integer>();

    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    m_liftingUnitTreeSet.add(RobotMap.LIFTING_UNIT_HEIGHT_START);
    // m_liftingUnitPositions = (Integer[])m_liftingUnitTreeSet.toArray();
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
    return manualHoldOffset.getDouble(0.1);
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
    return Motors.liftMaster.getSelectedSensorPosition();
    // return encoderConverter.getDistance(Motors.liftMaster.getSelectedSensorPosition());
  }

  /**
   * Sets the target position in mm
   */
  public void setTargetPosition(double position) {
    m_practicalTargetPosition = Algorithms.limit(position, 0, RobotMap.LIFTING_UNIT_DRIVE_LENGTH);
  }

  public void setTargetPosition(EPositions direction) {
    switch(direction) {

      case next: {
        if(m_liftingUnitPosition + 1 < m_liftingUnitPositions.length) {
          setTargetPosition(m_liftingUnitPositions[++m_liftingUnitPosition]);
        }
      } break;

      case hold: {
        setTargetPosition(m_liftingUnitPosition);
      } break;

      case previous: {
        if(m_liftingUnitPosition > 0) {
          setTargetPosition(m_liftingUnitPositions[--m_liftingUnitPosition]);
        }
      }
    }
  }

  public double getTargetPosition() {
    return m_practicalTargetPosition;
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
    Motors.liftMaster.set(ControlMode.MotionMagic, getTargetPosition());
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

  public boolean isDrivePermitted(double targetPosition, double cartTargetPosition) {
    this.m_theoreticalTargetPosition = targetPosition;
    if(getPosition() > RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT && !Robot.cart.isTargetPositionReached()) {
      m_practicalTargetPosition = RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT - RobotMap.LIFTING_UNIT_SAFETY_ZONE / 2;
      return true;
    }

    if(targetPosition >= RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT - RobotMap.LIFTING_UNIT_SAFETY_ZONE) {
      if(targetPosition <= RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT) {
        m_practicalTargetPosition = m_theoreticalTargetPosition;
        lifting_unit_drive_permitted = true;
      } else if(Robot.cart.isTargetPositionReached()) {
        m_practicalTargetPosition = m_theoreticalTargetPosition;
        lifting_unit_drive_permitted = true;
      } else {
        lifting_unit_drive_permitted = false;
      }
    } else {
      if(Robot.cart.getPosition() >= RobotMap.CART_FORWARD_SAFETY_LENGHT && cartTargetPosition >= RobotMap.CART_FORWARD_SAFETY_LENGHT) {
        m_practicalTargetPosition = m_theoreticalTargetPosition;
        lifting_unit_drive_permitted = true;
      } else if(Robot.cart.getPosition() <= RobotMap.CART_REVERSE_SAFETY_LENGHT && cartTargetPosition <= RobotMap.CART_REVERSE_SAFETY_LENGHT) {
        m_practicalTargetPosition = m_theoreticalTargetPosition;
        lifting_unit_drive_permitted = true;
      } else if(Robot.cart.getPosition() >= RobotMap.CART_FORWARD_SAFETY_LENGHT && cartTargetPosition <= RobotMap.CART_FORWARD_SAFETY_LENGHT) {
        m_practicalTargetPosition = RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT - RobotMap.LIFTING_UNIT_SAFETY_ZONE / 2;
        lifting_unit_drive_permitted = true;
      } else if(Robot.cart.getPosition() <= RobotMap.CART_FORWARD_SAFETY_LENGHT && cartTargetPosition >= RobotMap.CART_FORWARD_SAFETY_LENGHT) {
        m_practicalTargetPosition = RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT - RobotMap.LIFTING_UNIT_SAFETY_ZONE / 2;
        lifting_unit_drive_permitted = true;
      } else if(Robot.cart.isTargetPositionReached()) {
        m_practicalTargetPosition = m_theoreticalTargetPosition;
        lifting_unit_drive_permitted = true;
      } else {
        System.out.println("Lifting Unit System Failed");
      }
    }
    return lifting_unit_drive_permitted;
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
      Motors.liftFollower.setSelectedSensorPosition(0);
      m_liftingUnitPosition = 0;
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
    builder.addDoubleProperty("Motor Speed", Motors.liftMaster::getMotorOutputPercent, Motors.liftMaster::set);
    builder.addBooleanProperty("Reverse Limit", Motors.liftMaster.getSensorCollection()::isRevLimitSwitchClosed, null);
    builder.addBooleanProperty("Forward limit", Motors.liftMaster.getSensorCollection()::isFwdLimitSwitchClosed, null);
    builder.addBooleanProperty("Zeroed", this::isZeroed, null);
    builder.addDoubleProperty("Position (mm)", this::getPosition, null);
    builder.addDoubleProperty("Position raw (pulses)", Motors.liftMaster::getSelectedSensorPosition, null);
    builder.addDoubleProperty("Target position (mm)", this::getTargetPosition, this::setTargetPosition);
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
