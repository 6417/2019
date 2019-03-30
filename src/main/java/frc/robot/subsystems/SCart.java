/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

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
  boolean m_isHomed = false, m_autonomous = false;
  public boolean drive_manual = false;
  public boolean drive_autonomous = false; 
  int m_targetPosition = 0;
  TreeMap<Integer, Integer> cartLiftProfile = new TreeMap<Integer, Integer>();

  /**
   * Limit switches of the cart are connected to a remote Talon SRX
   */
  FridolinsTalonSRX remoteTalon = new FridolinsTalonSRX(RobotMap.CART_REMOTE_LIMIT_SWITCH_ID);

  public SCart() {
    setSubsystem("Cart");
    addChild(Motors.cartMotor);
    resetSubsystem();
    initializeCartLiftProfile();
  }

  private void resetSubsystem() {
    m_isHomed = false;
    m_targetPosition = 0;
    m_autonomous = false;
  }

  private void initializeCartLiftProfile() {
    cartLiftProfile.put(0, RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT);
    cartLiftProfile.put(RobotMap.CART_REVERSE_SAFETY_LENGTH, RobotMap.LIFTING_UNIT_SAFETY_HEIGHT);
    cartLiftProfile.put(RobotMap.CART_FORWARD_SAFETY_LENGTH, RobotMap.LIFTING_UNIT_MINIMUM_HEIGHT);
    cartLiftProfile.put(RobotMap.CART_DRIVE_LENGTH, 0);
  }

  public boolean isHomed() {
    return m_isHomed;
  }

  private int calculateNextPosition(int targetPosition) {
    int driveToPosition = targetPosition;
    Integer currentProfileSegment = cartLiftProfile.floorKey((int)getPosition());
    Integer targetProfileSegment = cartLiftProfile.floorKey(targetPosition);

    if(currentProfileSegment == null || targetProfileSegment == null) {
      System.out.println("SCart: Segments not found");
      return (int)getPosition();
    }

    // contains all points in the profile which has to be driven
    NavigableMap<Integer, Integer> motionProfile;
    if(targetProfileSegment > currentProfileSegment) {
      motionProfile = cartLiftProfile.subMap(currentProfileSegment, true, targetProfileSegment, true);
    } else {
      motionProfile = cartLiftProfile.subMap(targetProfileSegment, true, currentProfileSegment, true);
    }

    // calculating the minimum height in the overall profile. It could be changed to only check the current and next segment's height.
    int minimumLiftHeight = 0;
    for(Map.Entry<Integer, Integer> segment : motionProfile.entrySet()) {
      minimumLiftHeight = Math.max(segment.getValue(), minimumLiftHeight);
    }
    Robot.liftingUnit.setMinimumHeight(minimumLiftHeight);

    // if(Math.abs(targetPosition - getPosition()) > RobotMap.CART_DRIVE_LENGTH / 2) {
    //   Robot.liftingUnit.setMaximumHeight(RobotMap.LIFTING_UNIT_SAFETY_HEIGHT);
    // }
    
    // if(Robot.liftingUnit.getPosition() > RobotMap.LIFTING_UNIT_SAFETY_HEIGHT + RobotMap.LIFTING_UNIT_SAFETY_ZONE) {
    //   System.out.println("SCart: Not below maximum height. It's now " + Robot.liftingUnit.getPosition() + " and should be " + (RobotMap.LIFTING_UNIT_SAFETY_HEIGHT + RobotMap.LIFTING_UNIT_SAFETY_ZONE));
    //   return (int)getPosition();
    // }
    
    // if current lift position is below the required height, only drive up to the
    // next segment's point.
    // if current lift position is above the required height, one can drive up to
    // the target position.
    if (Robot.liftingUnit.getPosition() < minimumLiftHeight - RobotMap.LIFTING_UNIT_SAFETY_ZONE) {
      // check if target is in front or behind current position
      if (targetPosition > getPosition()) {
        // get next segment in the profile that is higher than the current position
        Integer nextPoint = motionProfile.higherKey((int) getPosition());
        if (nextPoint != null) {
          driveToPosition = nextPoint;
        }
      } else {
        driveToPosition = currentProfileSegment;
      }
    }

    return driveToPosition;
  } 

  public void enableAutonomous(boolean enable) {
    m_autonomous = enable;
  }

  /**
   * Returns position of the cart in mm.
   * 
   * @return Position in mm
   */
  public double getPosition() {
    // return encoderConverter.getDistance(Motors.cartMotor.getSelectedSensorPosition());
    return Motors.cartMotor.getSelectedSensorPosition();
  }

  /**
   * Moves the cart to the desired encoder position.
   * @param targetPos Position of the cart in encoder ticks
   */
  public void setPosition(int targetPos) {
    m_targetPosition = Algorithms.limit(targetPos, 0, RobotMap.CART_DRIVE_LENGTH);
  }

  public void drive(double value) {
    if (m_autonomous) {
      driveAutonomous();
    } else {
      driveManual(value);
    }
  }

  public void drive() {
    drive(0);
  }

  private void driveAutonomous() {
    // when the system is not homed, do not drive the cart!
    if (!isHomed()) {
      stop();
      return;
    }

    drive_autonomous = true;

    int targetPosition = calculateNextPosition(m_targetPosition);

    Motors.cartMotor.set(ControlMode.MotionMagic, targetPosition);
  }

  private void driveManual(double speed) {
    if (Robot.hatchGripper.isExtended() && speed >= 0 && getPosition() >= RobotMap.CART_DRIVE_LENGTH_HATCH_MM - 20) {
      speed = 0;
    }
    Motors.cartMotor.set(ControlMode.PercentOutput, speed);
    drive_manual = true;
  }

  /**
   * Stops the motor for the cart.
   */
  public void stop() {
    Motors.cartMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Check Limit switches and set the encoder to the specific encoder ticks
   */
  public void checkLimitSwitches() {
    if (!remoteTalon.getSensorCollection().isRevLimitSwitchClosed()) {
      Motors.cartMotor.setSelectedSensorPosition(0);
      m_isHomed = true;
    } else if (!remoteTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
      Motors.cartMotor.setSelectedSensorPosition(RobotMap.CART_DRIVE_LENGTH);
      m_isHomed = true;
    }
  }

  /**
   * Runs a automatic calibration routine to find it's zero point.
   * 
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

   /**
   * Checks if cart's position is in the desired range
   * 
   * @param position position to reach
   * @return true, when current position is in the range, false when not in range
   */
  public boolean isInRange(int position) {
    return (Math.abs(getPosition() - position) < RobotMap.CART_POSITION_ZONE);
  }

  @Override
  public void periodic() {
    checkLimitSwitches();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setActuator(true);
    builder.setSafeState(this::stop);
    builder.addDoubleProperty("Motor Speed", Motors.cartMotor::get, Motors.cartMotor::set);
    builder.addBooleanProperty("Reverse Limit", remoteTalon.getSensorCollection()::isRevLimitSwitchClosed, null);
    builder.addBooleanProperty("Forward limit", remoteTalon.getSensorCollection()::isFwdLimitSwitchClosed, null);
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addBooleanProperty("In Front Window", this::isInFrontWindow, null);
    builder.addBooleanProperty("In Back Window", this::isInBackWindow, null);
  }

  @Override
  protected void initDefaultCommand() {
    // setDefaultCommand(new CLiftingUnitOrderdHeight());
  }

}
