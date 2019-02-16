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

  //Subsystems
  public static final boolean CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE = false;
  public static final boolean HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE = false;
  public static final boolean LIFTING_UNIT_SUBSYSTEM_IS_IN_USE = false;
  public static final boolean SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE = false;
  public static final boolean CART_SUBYSTEM_IS_IN_USE = false;

  // Test Subsystems
  public static final boolean CART_TESTSUBYSTEM_IS_IN_USE = true;

  //TODO change Motor IDs

  //Motors
  public static final int CARGO_GRIPPER_MOTOR_RIGHT_ID = 1;
  public static final int CARGO_GRIPPER_MOTOR_LEFT_ID = 2;
  public static final int HATCH_GRIPPER_MOTOR_ID = 3;
  public static final int LIFTING_UNIT_MOTOR_LEFT_ID = 50;
  public static final int LIFTING_UNIT_MOTOR_RIGHT_ID = 51;
  public static final int CART_MOTOR_ID                 = 52;

  public static final int SWERVE_DRIVE_FRONT_LEFT_ID    = 32;
  public static final int SWERVE_DRIVE_FRONT_RIGHT_ID   = 38;
  public static final int SWERVE_DRIVE_BACK_LEFT_ID     = 34;
  public static final int SWERVE_DRIVE_BACK_RIGHT_ID    = 36;
  public static final int SWERVE_ANGLE_FRONT_LEFT_ID    = 33;
  public static final int SWERVE_ANGLE_FRONT_RIGHT_ID   = 39;
  public static final int SWERVE_ANGLE_BACK_LEFT_ID     = 35;
  public static final int SWERVE_ANGLE_BACK_RIGHT_ID    = 37;

  //Joysticks
  public static final int JOYSTICK_MAIN_DRIVER_ID = 0;
  public static final int JOYSTICK_SUPPORT_DRIVER_ID = 1;

  //JoystickButtons
  public static final int CARGO_GRIPPER_BUTTON_PULL_ID = 1;
  public static final int CARGO_GRIPPER_BUTTON_PUSH_ID = 2;
  public static final int HATCH_GRIPPER_BUTTON_EXTEND_ID = 3;
  public static final int HATCH_GRIPPER_BUTTON_RETRACT_ID = 4;
  public static final int SWERVE_ANGLE_CALIBRATE_BUTTON_ID = 5;
  public static final int FIELD_ANGLE_RESET_BUTTON_ID = 6;


  //Speeds
  public static final double SPEED_MULTIPLIER = 1;
  public static final double DRIVE_SPEED_MULITPLIER = 1;
  public static final double TURN_SPEED_MULTIPLIER = 1;

  public static final double STOP_SPEED = 0;

  public static final double CARGO_GRIPPER_SPEED = 0.1;
  public static final double HATCH_GRIPPER_SPEED = 0.1;

  //Deadzone
  public static final double DEADZONE_RANGE = 0.1;

  //Field Oriented Drive
  public static double DEFAULT_FIELD_ANGLE = 0;

  //Encoders
  /**
   * mm per pulse
   */
  public static final int CART_ENCODER_DISTANCE_PER_PULSE = 1;
  /**
   * Cart motor velocity in units/100ms
   */
  public static final int CART_ENCODER_UNITS_PER_100_MS = 38000;
  /**
   * Cart length in mm measured from the zero point.
   */
  public static final int CART_DRIVE_LENGTH = 200000;
  /**
   * Window in which the cart should drive slowlier.
   */
  public static final int CART_WINDOW_LENGTH = 20000;

  //TODO change Encodertick values for Swerve Drive

  //Robot
  public static final int SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT = 11564;
  public static final int SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT = 196608;
  public static final double WHEEL_DISTANCE_LENGTH = 64;
  public static final double WHEEL_DISTANCE_WIDTH = 61;

}
