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

  //Not sorted Variables
  public static final int CART_REVERSE_SAFETY_LENGTH = 2000;
  public static final int CART_FORWARD_SAFETY_LENGTH = 9000;
  public static final int CART_POSITION_ZONE = 200;
  public static final int CART_CENTER_POINT = 2000;
  public static final int LIFTING_UNIT_SAFETY_HEIGHT = 1000;
  public static final int LIFTING_UNIT_SAFETY_ZONE = 200;

  // Subsystems
  public static final boolean CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE     = true;
  public static final boolean HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE     = true;
  public static final boolean LIFTING_UNIT_SUBSYSTEM_IS_IN_USE      = true;
  public static final boolean CART_SUBSYSTEM_IS_IN_USE              = true;
  public static final boolean SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE      = true;
  public static final boolean ROBOT_ELEVATOR_SUBSYSTEM_IN_USE       = true;

  // Motors
  public static final int SWERVE_DRIVE_FRONT_LEFT_ID    = 32;
  public static final int SWERVE_ANGLE_FRONT_LEFT_ID    = 33;
  public static final int SWERVE_DRIVE_BACK_LEFT_ID     = 34;
  public static final int SWERVE_ANGLE_BACK_LEFT_ID     = 35;
  public static final int SWERVE_DRIVE_BACK_RIGHT_ID    = 36;
  public static final int SWERVE_ANGLE_BACK_RIGHT_ID    = 37;
  public static final int SWERVE_DRIVE_FRONT_RIGHT_ID   = 38;
  public static final int SWERVE_ANGLE_FRONT_RIGHT_ID   = 39;

  public static final int HATCH_GRIPPER_MOTOR_ID        = 47;

  public static final int CARGO_GRIPPER_MOTOR_LEFT_ID   = 48;
  public static final int CARGO_GRIPPER_MOTOR_RIGHT_ID  = 49;

  public static final int LIFTING_UNIT_MOTOR_LEFT_ID    = 50;
  public static final int LIFTING_UNIT_MOTOR_RIGHT_ID   = 51;

  public static final int CART_MOTOR_ID                 = 52;
  public static final int CART_REMOTE_LIMIT_SWITCH_ID   = SWERVE_DRIVE_FRONT_RIGHT_ID;

  public static final int ROBOT_ELEVATOR_MOTOR_LEFT_ID  = 53;
  public static final int ROBOT_ELEVATOR_MOTOR_RIGHT_ID = 54;

  // DIO
  public static final int HATCH_GRIPPER_DIO_TOP = 0;
  public static final int HATCH_GRIPPER_DIO_RIGHT = 1;
  public static final int HATCH_GRIPPER_DIO_LEFT = 2;

  // Joysticks
  public static final int JOYSTICK_MAIN_DRIVER_ID = 0;
  public static final int JOYSTICK_SUPPORT_DRIVER_ID = 1;
  public static final int JOYSTICK_HEIGHT_CONTROLLER_ID = 2;

  // JoystickButtons
    //Triggers
    public static final int SUPPORT_CARGO_GRIPPER_BUTTON_PULL_ID = 8;
    public static final int SUPPORT_CARGO_GRIPPER_BUTTON_PUSH_ID = 7;

    public static final int SUPPORT_HATCH_GRIPPER_BUTTON_EXTEND_ID = 6;
    public static final int SUPPORT_HATCH_GRIPPER_BUTTON_RETRACT_ID = 5;

    //Color Buttons
    public static final int SUPPORT_HATCH_GRIPPER_BUTTON_PRESS_HATCH = 4;
    public static final int SUPPORT_HATCH_PICK_UP_BUTTON = 2;
    public static final int SUPPORT_START_POS_BUTTON = 3;
    public static final int SUPPORT_CARGO_GROUND_BUTTON = 1;

    public static final int HEIGHT_HATCH_STATION_BUTTON_ID = 2;
    public static final int HEIGHT_HATCH_MID_BUTTON_ID = 3;
    public static final int HEIGHT_HATCH_TOP_BUTTON_ID = 4;
    public static final int HEIGHT_CARGO_GROUND_BUTTON_ID = 1;

    //Calibrate Buttons
    public static final int SUPPORT_HATCH_GRIPPER_BUTTON_CALIBRATE_ID = 9;
    public static final int SUPPORT_LIFTING_UNIT_BUTTON_CALIBRATE_ID = 10;

    //POV Channel
    public static final int SUPPORT_POV_CHANNEL_ID = 0;

    //Height Controller
    //Buttons for Hatch heights
    public static final int HEIGHT_CONTROLLER_HATCH_STATION = 1;
    public static final int HEIGHT_CONTROLLER_HATCH_ROCKET_2 = 2;
    public static final int HEIGHT_CONTROLLER_HATCH_ROCKET_3 = 3;
    public static final int HEIGHT_CONTROLLER_CARGO_BOTTOM = 4;

    //POV for Height Controller for Cargo heights
    public static final int HEIGHT_CONTROLLER_CARGO_ROCKET_1 = 180;
    public static final int HEIGHT_CONTROLLER_CARGO_ROCKET_2 = 90;
    public static final int HEIGHT_CONTROLLER_CARGO_ROCKET_3 = 0;
    public static final int HEIGHT_CONTROLLER_CARGO_STATION = 270;

  public static final int MAIN_SWERVE_ANGLE_CALIBRATE_BUTTON_ID = 9;
  public static final int MAIN_FIELD_ANGLE_RESET_BUTTON_ID = 10;
  public static final int MAIN_FIELD_ANGLE_ENABLE_BUTTON_ID = 2;
  public static final int MAIN_FIELD_ANGLE_DISABLE_BUTTON_ID = 3;

  public static final int MAIN_SWERVE_BOOST_BUTTON_ID = 7;
  public static final int MAIN_SWERVE_SLOW_MODE_BUTTON_ID = 8;
  public static final int MAIN_SWERVE_HATCH_ORIENTED_BUTTON_ID = 3;
  public static final int MAIN_SWERVE_CARGO_ORIENTED_BUTTON_ID = 4;
  public static final int MAIN_SWERVE_FIELD_ORINETED_BUTTON_ID = 2;

  // General Speed Variables
  public static final double SPEED_MULTIPLIER = 1;
  public static final double STOP_SPEED = 0;

  // Swerve Speed Variables
  public static final double DRIVE_SPEED_MULITPLIER = 0.45;
  public static final double TURN_SPEED_MULTIPLIER = 0.65;
  public static final double SWERVE_SPEED_BOOST = 1;
  public static final double SWERVE_BULLET_TIME = 0.35;
  public static final double SWERVE_CALIBRATE_SPEED = 0.2;

  // Gripper Speed Variables
  public static final double CARGO_GRIPPER_SPEED = 0.3;
  public static final double HATCH_CALIBRATE_SPEED = -0.2;

  // Deadzone
  public static final double DEADZONE_RANGE = 0.05;

  // Field Oriented Drive
  public static double DEFAULT_FIELD_ANGLE = 0;

  // Lifting Unit
      public static final int LIFTING_UNIT_ZERO_POSITION = 1355;
      /** Lifting Unit motor max velocity in units/100ms */
      public static final int LIFTING_UNIT_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS = 1200;
      /** mm per pulse */
      public static final int LIFTING_UNIT_DISTANCE_PER_PULSE = 1;
      /** Lifting unit motor velocity in units/100ms */
      public static final int LIFTING_UNIT_ENCODER_UNITS_PER_100_MS = 38000;
      /** Lifting unit length in encoder pulses */
      public static final int LIFTING_UNIT_DRIVE_LENGTH = 33505;
      /** Lifting unit minimum height such that the cargo gripper is not colliding with the cart system */
      public static final int LIFTING_UNIT_MINIMUM_HEIGHT = 1500;
      /** Lifting unit positions for driving and starting... */
      public static final int LIFTING_UNIT_HEIGHT_START = 3198;
      /** Lifting unit positions for hatch Station, the ship and the first rocket level  */
      public static final int LIFTING_UNIT_HEIGHT_HATCH_STATION = 2600;
      /** Lifting unit other hatch positions for rocket */
      public static final int LIFTING_UNIT_HEIGHT_HATCH_BOTTOM = 3198;
      public static final int LIFTING_UNIT_HEIGHT_HATCH_MID = 14695;
      public static final int LIFTING_UNIT_HEIGHT_HATCH_TOP = 26900;
      /** Lifting unit cargo station on field and driverstation */
      public static final int LIFTING_UNIT_HEIGHT_CARGO_STATION = 17000;
      public static final int LIFTING_UNIT_HEIGHT_CARGO_DEPOT = 1500;
      /** Lifting unit cargo rocket stations */
      public static final int LIFTING_UNIT_HEIGHT_CARGO_BOTTOM = 9910;
      public static final int LIFTING_UNIT_HEIGHT_CARGO_MID = 21960;
      public static final int LIFTING_UNIT_HEIGHT_CARGO_TOP = 33200;
      /** Lifting unit cargo cargo ship height */
      public static final int LIFTING_UNIT_HEIGHT_CARGO_SHIP = 16700;
    

  // Hatch

      /** Hatch motor max velocity in units/100ms */
      public static final int HATCH_GRIPPER_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS = 33000;
      /** Hatch extended length in encoder pulses */
      public static final int HATCH_DRIVE_EXTENDED = 14000;
      /** Hatch retracted length in encoder pulses */
      public static final int HATCH_DRIVE_RETRACTED = 0;

  //Cart

      /** Cart motor max velocity in units/100ms */
      public static final int CART_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS = 2200;
      /** Cart length in mm measured from the zero point */
      public static final double CART_DRIVE_LENGTH_MM = 448;
      /** Cart length in encoder pulse */
      public static final int CART_DRIVE_LENGTH = 15620;
      /** Cart length in mm measured from the zero point. */
      public static final double CART_DRIVE_LENGTH_HATCH_MM = 70;
      /** Cart middle position in mm */
      public static final double CART_DRIVE_LENGTH_MID_MM = 200;
      /** Cart back position in mm */
      public static final double CART_DRIVE_LENGTH_BACK_MM = 0;
      /** Window in mm in which the cart should drive slowlier. */
      public static final int CART_WINDOW_LENGTH = 50;
      /** mm per pulse */
      public static final double CART_ENCODER_DISTANCE_PER_PULSE = CART_DRIVE_LENGTH_MM / CART_DRIVE_LENGTH;

  //Swerve
      
      // TODO change Encodertick values for Swerve Drive
      /** one rotation around Y axis of the swerve (drive) in encoder pulses */
      public static final int SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT = 11564;
      /** One rotation around Z axis of the swerve (steering) in encoder pulses */
      public static final int SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT = 196608;
      /** wheel to wheel distance in Y axis (cm) */
      public static final double WHEEL_DISTANCE_LENGTH = 64;
      /** wheel to wheel distance in x axis (cm) */
      public static final double WHEEL_DISTANCE_WIDTH = 61;

  // Robot

}
