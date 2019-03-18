/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drive.navx.CNavXReset;
import frc.robot.commands.drive.swerve.CSwerveSpeedMultiplier;
import frc.robot.commands.gripper.cargo.CCargoGripperPull;
import frc.robot.commands.gripper.cargo.CCargoGripperPush;
import frc.robot.commands.gripper.hatch.ConditionalHatchCommand;
import frc.robot.commands.groups.CHatchGrab;
import frc.robot.commands.groups.CHatchGripperCalibrate;
import frc.robot.commands.groups.CHatchHandOut;
import frc.robot.commands.groups.CHatchPress;
import frc.robot.commands.groups.CSwerveCalibrate;

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
  public static JoystickButton HatchGripperButtonPress;
  public static JoystickButton HatchGripperButtonCalibrate;
  public static JoystickButton CartButtonPressHatch;
  public static JoystickButton SwerveCalibrateButton;
  public static JoystickButton NavXResetButton;
  public static JoystickButton SwerveSpeedBoostButton;
  public static JoystickButton SwerveSpeedBulletTimeButton;

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
      CargoGripperButtonPull = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_CARGO_GRIPPER_PULL_AXIS_ID);
      CargoGripperButtonPush = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_CARGO_GRIPPER_PUSH_AXIS_ID);

      //Call Commands
      CargoGripperButtonPull.toggleWhenPressed(new CCargoGripperPull());
      CargoGripperButtonPush.toggleWhenPressed(new CCargoGripperPush());

    }

    //Initialize JoystickButtons when Subystem is in use
    if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      HatchGripperButtonExtend = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_EXTEND_ID);
      HatchGripperButtonRetract = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_RETRACT_ID);
      HatchGripperButtonPress = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_PRESS_HATCH);
      HatchGripperButtonCalibrate = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_CALIBRATE_ID);
      

      //Call Commands
      HatchGripperButtonExtend.whenPressed(new ConditionalHatchCommand(new CHatchGrab()));
      HatchGripperButtonRetract.whenPressed(new ConditionalHatchCommand(new CHatchHandOut()));
      HatchGripperButtonPress.whenPressed(new ConditionalHatchCommand(new CHatchPress()));
      HatchGripperButtonCalibrate.whenPressed(new ConditionalHatchCommand(new CHatchGripperCalibrate()));
    }

    if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      SwerveCalibrateButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_ANGLE_CALIBRATE_BUTTON_ID);
      NavXResetButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_FIELD_ANGLE_RESET_BUTTON_ID);
      SwerveSpeedBoostButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_BOOST_BUTTON_ID);
      SwerveSpeedBulletTimeButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_SLOW_MODE_BUTTON_ID);

      SwerveCalibrateButton.whenPressed(new CSwerveCalibrate());
      NavXResetButton.whenPressed(new CNavXReset());
      SwerveSpeedBoostButton.whileHeld(new CSwerveSpeedMultiplier(RobotMap.SWERVE_SPEED_BOOST));
      SwerveSpeedBulletTimeButton.whileHeld(new CSwerveSpeedMultiplier(RobotMap.SWERVE_BULLET_TIME));
    }

    if(RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      CartButtonPressHatch = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_PRESS_HATCH);
      CartButtonPressHatch.whenPressed(new CHatchPress());
    }

  }
}
