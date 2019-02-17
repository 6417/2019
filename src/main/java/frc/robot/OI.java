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
import frc.robot.commands.CHatchGripperExtend;
import frc.robot.commands.CHatchGripperRetract;
import frc.robot.commands.CNavXReset;
import frc.robot.commands.CSwerveCalibrate;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //Create Joystick
  public static Joystick JoystickMainDriver = new Joystick(RobotMap.JOYSTICK_MAIN_DRIVER_ID);
  public static Joystick JoystickSupportDriver = new Joystick(RobotMap.JOYSTICK_SUPPORT_DRIVER_ID);

  //Create JoystickButtons
  public static JoystickButton CargoGripperButtonPush;
  public static JoystickButton CargoGripperButtonPull;
  public static JoystickButton HatchGripperButtonExtend;
  public static JoystickButton HatchGripperButtonRetract;
  public static JoystickButton SwerveCalibrateButton;
  public static JoystickButton NavXResetButton;

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
      CargoGripperButtonPull = new JoystickButton(JoystickMainDriver, RobotMap.CARGO_GRIPPER_BUTTON_PULL_ID);
      CargoGripperButtonPush = new JoystickButton(JoystickMainDriver, RobotMap.CARGO_GRIPPER_BUTTON_PUSH_ID);

      //Call Commands
      CargoGripperButtonPull.toggleWhenPressed(new CCargoGripperPull());
      CargoGripperButtonPush.toggleWhenPressed(new CCargoGripperPush());

    }

    //Initialize JoystickButtons when Subystem is in use
    if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      HatchGripperButtonExtend = new JoystickButton(JoystickMainDriver, RobotMap.HATCH_GRIPPER_BUTTON_EXTEND_ID);
      HatchGripperButtonRetract = new JoystickButton(JoystickMainDriver, RobotMap.HATCH_GRIPPER_BUTTON_RETRACT_ID);

      //Call Commands
      //TODO write the code for Command group whitch requires the cartSubsystem
        HatchGripperButtonExtend.toggleWhenPressed(new CHatchGripperExtend());
        HatchGripperButtonRetract.toggleWhenPressed(new CHatchGripperRetract());
      
    }

    if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      SwerveCalibrateButton = new JoystickButton(JoystickMainDriver, RobotMap.SWERVE_ANGLE_CALIBRATE_BUTTON_ID);
      NavXResetButton = new JoystickButton(JoystickMainDriver, RobotMap.FIELD_ANGLE_RESET_BUTTON_ID);

      SwerveCalibrateButton.whenPressed(new CSwerveCalibrate());
      NavXResetButton.whenPressed(new CNavXReset());
    }
  }

}
