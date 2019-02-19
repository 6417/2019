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
import frc.robot.commands.CHatchGripperCalibrate;
import frc.robot.commands.CHatchGripperExtend;
import frc.robot.commands.CHatchGripperRetract;
import frc.robot.commands.CNavXReset;
import frc.robot.commands.CSwerveCalibrate;
import frc.robot.commands.CSwerveSpeedMultiplier;

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
  public static JoystickButton HatchGripperButtonCalibrate;
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
      CargoGripperButtonPull = new JoystickButton(JoystickMainDriver, RobotMap.SUPPORT_CARGO_GRIPPER_PULL_AXIS_ID);
      CargoGripperButtonPush = new JoystickButton(JoystickMainDriver, RobotMap.SUPPORT_CARGO_GRIPPER_PUSH_AXIS_ID);

      //Call Commands
      CargoGripperButtonPull.toggleWhenPressed(new CCargoGripperPull());
      CargoGripperButtonPush.toggleWhenPressed(new CCargoGripperPush());

    }

    //Initialize JoystickButtons when Subystem is in use
    if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      HatchGripperButtonExtend = new JoystickButton(JoystickMainDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_EXTEND_ID);
      HatchGripperButtonRetract = new JoystickButton(JoystickMainDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_RETRACT_ID);
      HatchGripperButtonCalibrate = new JoystickButton(JoystickMainDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_CALIBRATE_ID);

      //Call Commands
      //TODO write the code for Command group whitch requires the cartSubsystem
        HatchGripperButtonExtend.toggleWhenPressed(new CHatchGripperExtend());
        HatchGripperButtonRetract.toggleWhenPressed(new CHatchGripperRetract());
        HatchGripperButtonCalibrate.whenPressed(new CHatchGripperCalibrate());
        
    }

    if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      SwerveCalibrateButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_ANGLE_CALIBRATE_BUTTON_ID);
      NavXResetButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_FIELD_ANGLE_RESET_BUTTON_ID);
      SwerveSpeedBoostButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_BOOST_BUTTON_ID);
      SwerveSpeedBulletTimeButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_SLOW_MODE_BUTTON_ID);

      SwerveCalibrateButton.whenPressed(new CSwerveCalibrate());
      NavXResetButton.whenPressed(new CNavXReset());
      SwerveSpeedBoostButton.whileActive(new CSwerveSpeedMultiplier(RobotMap.SWERVE_SPEED_BOOST));
      SwerveSpeedBulletTimeButton.whileActive(new CSwerveSpeedMultiplier(RobotMap.SWERVE_BULLET_TIME));
    }

  }

}
