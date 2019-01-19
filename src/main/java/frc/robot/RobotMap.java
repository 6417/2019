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

  //Motors
  public static final int CARGO_GRIPPER_MOTOR_RIGHT_ID = 1;
  public static final int CARGO_GRIPPER_MOTOR_LEFT_ID = 2;

  //Joysticks
  public static final int JOYSTICK_GRIPPER_SYSTEMS_ID = 0;

  //JoystickButtons
  public static final int CARGO_GRIPPER_BUTTON_PULL_ID = 1;
  public static final int CARGO_GRIPPER_BUTTON_PUSH_ID = 2;

}
