/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.CCargoGripperPull;
import frc.robot.commands.CCargoGripperPush;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //Create Joystick
  public static Joystick JoystickGripperSystems = new Joystick(RobotMap.JOYSTICK_GRIPPER_SYSTEMS_ID);

  //Create JoystickButtons
  public static JoystickButton CargoGripperButtonPush;
  public static JoystickButton CargoGripperButtonPull;

  private static OI INSTANCE;

  public static OI getInstance() {
    if(INSTANCE == null) {
      INSTANCE = new OI();
    }
    return INSTANCE;
  }
  
  private OI() {

    //Initialize JoystickButtons when Subystem is in use
    if(RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      CargoGripperButtonPull = new JoystickButton(JoystickGripperSystems, RobotMap.CARGO_GRIPPER_BUTTON_PULL_ID);
      CargoGripperButtonPush = new JoystickButton(JoystickGripperSystems, RobotMap.CARGO_GRIPPER_BUTTON_PUSH_ID);

      //Call Commands
      CargoGripperButtonPull.toggleWhenPressed(new CCargoGripperPull());
      CargoGripperButtonPush.toggleWhenPressed(new CCargoGripperPush());

    }
  }

}
