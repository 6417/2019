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
  public static final boolean CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE = true;
  public static final boolean HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE = true;

  //TODO change Motor IDs

  //Motors
  public static final int CARGO_GRIPPER_MOTOR_RIGHT_ID = 1;
  public static final int CARGO_GRIPPER_MOTOR_LEFT_ID = 2;
  public static final int HATCH_GRIPPER_MOTOR_ID = 3;
  public static final int CART_MOTOR_ID = 2;

  //Joysticks
  public static final int JOYSTICK_MAIN_DRIVER_ID = 0;
  public static final int JOYSTICK_SUPPORT_DRIVER_ID = 1;

  //JoystickButtons
  public static final int CARGO_GRIPPER_BUTTON_PULL_ID = 1;
  public static final int CARGO_GRIPPER_BUTTON_PUSH_ID = 2;
  public static final int HATCH_GRIPPER_BUTTON_EXTEND_ID = 3;
  public static final int HATCH_GRIPPER_BUTTON_RETRACT_ID = 4;

  //Speeds
  public static final double STOP_SPEED = 0;

  public static final double CARGO_GRIPPER_SPEED = 0.1;
  public static final double HATCH_GRIPPER_SPEED = 0.1;

  //Encoders
  public static final double CART_ENCODER_DISTANCE_PER_PULSE = 1;

}
