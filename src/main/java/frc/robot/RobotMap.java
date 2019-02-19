/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // Subsystems
  public static final boolean CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE = true;
  public static final boolean HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE = false;
  public static final boolean LIFTING_UNIT_SUBSYSTEM_IS_IN_USE = true;
  public static final boolean SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE = true;
  public static final boolean CART_SUBYSTEM_IS_IN_USE = true;

  // Test Subsystems
  public static final boolean CART_TESTSUBYSTEM_IS_IN_USE = false;

  // TODO change Motor IDs

  //Motors
  public static final int CARGO_GRIPPER_MOTOR_RIGHT_ID  = 49;
  public static final int CARGO_GRIPPER_MOTOR_LEFT_ID   = 48;
  public static final int HATCH_GRIPPER_MOTOR_ID        = 47;

  public static final int LIFTING_UNIT_MOTOR_LEFT_ID    = 50;
  public static final int LIFTING_UNIT_MOTOR_RIGHT_ID   = 51;

  public static final int SWERVE_DRIVE_FRONT_LEFT_ID    = 32;
  public static final int SWERVE_DRIVE_FRONT_RIGHT_ID   = 38;
  public static final int SWERVE_DRIVE_BACK_LEFT_ID     = 34;
  public static final int SWERVE_DRIVE_BACK_RIGHT_ID    = 36;
  public static final int SWERVE_ANGLE_FRONT_LEFT_ID    = 33;
  public static final int SWERVE_ANGLE_FRONT_RIGHT_ID   = 39;
  public static final int SWERVE_ANGLE_BACK_LEFT_ID     = 35;
  public static final int SWERVE_ANGLE_BACK_RIGHT_ID    = 37;

  public static final int CART_MOTOR_ID                 = 52;
  public static final int CART_REMOTE_LIMIT_SWITCH_ID   = SWERVE_DRIVE_FRONT_RIGHT_ID;

  //DIO
  public static final int HATCH_GRIPPER_DIO_TOP = 0;
  public static final int HATCH_GRIPPER_DIO_RIGHT = 1;
  public static final int HATCH_GRIPPER_DIO_LEFT = 2;

  // Joysticks
  public static final int JOYSTICK_MAIN_DRIVER_ID = 0;
  public static final int JOYSTICK_SUPPORT_DRIVER_ID = 1;

  // JoystickButtons
  public static final int CARGO_GRIPPER_BUTTON_PULL_ID = -1;
  public static final int CARGO_GRIPPER_BUTTON_PULL_ID = 7;
  public static final int CARGO_GRIPPER_BUTTON_PUSH_ID = 5;
  public static final int HATCH_GRIPPER_BUTTON_EXTEND_ID = 3;
  public static final int HATCH_GRIPPER_BUTTON_RETRACT_ID = 4;
  public static final int HATCH_GRIPPER_BUTTON_CALIBRATE_ID = 2;
  public static final int SWERVE_ANGLE_CALIBRATE_BUTTON_ID = 9;
  public static final int FIELD_ANGLE_RESET_BUTTON_ID = 10;


  public static final double SPEED_MULTIPLIER = 1;
  public static final double DRIVE_SPEED_MULITPLIER = 0.6;
  public static final double TURN_SPEED_MULTIPLIER = 1;

  public static final double STOP_SPEED = 0;

  public static final double CARGO_GRIPPER_SPEED = 0.3;
  public static final double HATCH_CALIBRATE_SPEED = -0.4;
  public static final double SWERVE_CALIBRATE_SPEED = 0.4; 

  // Deadzone
  public static final double DEADZONE_RANGE = 0.05;

  // Field Oriented Drive
  public static double DEFAULT_FIELD_ANGLE = 0;

  // Encoders
  /**
   * Hatch motor max velocity in units/100ms
   */
  public static final int HATCH_GRIPPER_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS = 33000;
  /**
  * Hatch extended length in encoder pulses
  */
 public static final int HATCH_DRIVE_EXTENDED = 14000;
  /**
  * Hatch retracted length in encoder pulses
  */
  public static final int HATCH_DRIVE_RETRACTED = 0;

  /**
   * Cart motor max velocity in units/100ms
   */
  public static final int CART_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS = 2200;
  /**
   * Cart length in mm measured from the zero point.
   */
  public static final double CART_DRIVE_LENGTH_MM = 448;
  /**
   * Cart length in encoder pulses
   */
  public static final int CART_DRIVE_LENGTH = 15450;
  /**
   * Window in mm in which the cart should drive slowlier.
   */
  public static final int CART_WINDOW_LENGTH = 50;
  /**
  * mm per pulse
  */
  public static final double CART_ENCODER_DISTANCE_PER_PULSE = CART_DRIVE_LENGTH_MM/CART_DRIVE_LENGTH;
  /**
   * mm per pulse
   */
  public static final int LIFTING_UNIT_DISTANCE_PER_PULSE = 1;
  /**
   * Lifting unit motor velocity in units/100ms
   */
  public static final int LIFTING_UNIT_ENCODER_UNITS_PER_100_MS = 38000;
  /**
   * Lifting unit length in mm measured from the zero point.
   */
  public static final int LIFTING_UNIT_DRIVE_LENGTH = 10000;
  /**
   * Lifting unit minimum height such that the cargo gripper is not colliding with
   * the cart system
   */
  public static final int LIFTING_UNIT_MINIMUM_HEIGHT = 100;

  // TODO change Encodertick values for Swerve Drive

  // Robot
  public static final int SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT = 11564;
  public static final int SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT = 196608;
  public static final double WHEEL_DISTANCE_LENGTH = 64;
  public static final double WHEEL_DISTANCE_WIDTH = 61;

}
