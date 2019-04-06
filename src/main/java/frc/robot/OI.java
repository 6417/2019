/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.cart.CCartDriveManual;
import frc.robot.commands.drive.navx.CNavXReset;
import frc.robot.commands.drive.swerve.CSwerveChangeDriveMode;
import frc.robot.commands.drive.swerve.CSwerveSpeedMultiplier;
import frc.robot.commands.gripper.cargo.CCargoGripperPull;
import frc.robot.commands.gripper.cargo.CCargoGripperPush;
import frc.robot.commands.gripper.hatch.ConditionalHatchCommand;
import frc.robot.commands.groups.CAutonomousDeliver;
import frc.robot.commands.groups.CHatchGrab;
import frc.robot.commands.groups.CHatchGripperCalibrate;
import frc.robot.commands.groups.CHatchHandOut;
import frc.robot.commands.groups.CHatchPress;
import frc.robot.commands.groups.CLiftingUnitCalibrate;
import frc.robot.commands.groups.CSwerveCalibrate;
import frc.robot.subsystems.SSwerve.DriveMode;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //Create Joystick
  public static Joystick JoystickMainDriver = new Joystick(RobotMap.JOYSTICK_MAIN_DRIVER_ID);
  public static Joystick JoystickSupportDriver = new Joystick(RobotMap.JOYSTICK_SUPPORT_DRIVER_ID);
  public static Joystick JoystickHeightController = new Joystick(RobotMap.JOYSTICK_HEIGHT_CONTROLLER_ID);

  //Create JoystickButtons
  public static JoystickButton CargoGripperButtonPush;
  public static JoystickButton CargoGripperButtonPull;
  public static JoystickButton HatchGripperButtonExtend;
  public static JoystickButton HatchGripperButtonRetract;
  public static JoystickButton HatchGripperButtonPress;
  public static JoystickButton HatchGripperButtonCalibrate;
  public static JoystickButton CartButtonPressHatch;
  public static Button CartButtonDriveManual;
  public static JoystickButton SwerveCalibrateButton;
  public static JoystickButton NavXResetButton;
  public static JoystickButton SwerveSpeedBoostButton;
  public static JoystickButton SwerveSpeedBulletTimeButton;
  public static JoystickButton SwerveHatchOrientedButton;
  public static JoystickButton SwerveCargoOrientedButton;
  public static JoystickButton SwerveFieldOrientedButton;
  public static JoystickButton LiftingUnitButtonCalibrate;

  public static JoystickButton StartPositionButton;
  public static JoystickButton HatchPickUpButton;
  public static JoystickButton CargoGroundButton;

  public static JoystickButton HatchStation;
  public static JoystickButton HatchRocket2;
  public static JoystickButton HatchRocket3;
  public static JoystickButton CargoGroundButton2;
  public static Button CargoRocket1;
  public static Button CargoRocket2;
  public static Button CargoRocket3;
  public static Button CargoStation;

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
      CargoGripperButtonPull = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_CARGO_GRIPPER_BUTTON_PULL_ID);
      CargoGripperButtonPush = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_CARGO_GRIPPER_BUTTON_PUSH_ID);
      //Call Commands
      CargoGripperButtonPull.toggleWhenPressed(new CCargoGripperPull());
      CargoGripperButtonPush.whileHeld(new CCargoGripperPush());
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
      HatchGripperButtonCalibrate.whenPressed(new CHatchGripperCalibrate());
    }

    if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      SwerveCalibrateButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_ANGLE_CALIBRATE_BUTTON_ID);
      NavXResetButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_FIELD_ANGLE_RESET_BUTTON_ID);
      SwerveSpeedBoostButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_BOOST_BUTTON_ID);
      SwerveSpeedBulletTimeButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_SLOW_MODE_BUTTON_ID);
      SwerveHatchOrientedButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_HATCH_ORIENTED_BUTTON_ID);
      SwerveCargoOrientedButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_CARGO_ORIENTED_BUTTON_ID);
      SwerveFieldOrientedButton = new JoystickButton(JoystickMainDriver, RobotMap.MAIN_SWERVE_FIELD_ORINETED_BUTTON_ID);

      CSwerveChangeDriveMode hatchOriented = new CSwerveChangeDriveMode(DriveMode.HatchOriented);
      hatchOriented.setRunWhenDisabled(true);
      CSwerveChangeDriveMode cargoOriented = new CSwerveChangeDriveMode(DriveMode.CargoOriented);
      cargoOriented.setRunWhenDisabled(true);
      CSwerveChangeDriveMode fieldOriented = new CSwerveChangeDriveMode(DriveMode.FieldOriented);
      fieldOriented.setRunWhenDisabled(true);
      CNavXReset navxReset = new CNavXReset();
      navxReset.setRunWhenDisabled(true);

      SwerveCalibrateButton.whenPressed(new CSwerveCalibrate());
      NavXResetButton.whenPressed(navxReset);
      SwerveSpeedBoostButton.whileHeld(new CSwerveSpeedMultiplier(RobotMap.SWERVE_SPEED_BOOST));
      SwerveSpeedBulletTimeButton.whileHeld(new CSwerveSpeedMultiplier(RobotMap.SWERVE_BULLET_TIME));
      SwerveHatchOrientedButton.whenPressed(hatchOriented);
      SwerveCargoOrientedButton.whenPressed(cargoOriented);
      SwerveFieldOrientedButton.whenPressed(fieldOriented);

    }

    if(RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      CartButtonPressHatch = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_GRIPPER_BUTTON_PRESS_HATCH);
      CartButtonPressHatch.whenPressed(new CHatchPress());
      CartButtonDriveManual = new Button() {
       @Override
       public boolean get() {
         return (JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 90);
       }
      };
      CartButtonDriveManual.whileHeld(new CCartDriveManual());
    }

    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE && RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      StartPositionButton = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_START_POS_BUTTON);
      CargoGroundButton = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_CARGO_GROUND_BUTTON);
      HatchPickUpButton = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_HATCH_PICK_UP_BUTTON);
      LiftingUnitButtonCalibrate = new JoystickButton(JoystickSupportDriver, RobotMap.SUPPORT_LIFTING_UNIT_BUTTON_CALIBRATE_ID);
      
      StartPositionButton.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_START, RobotMap.CART_CENTER_POINT));
      CargoGroundButton.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_DEPOT, RobotMap.CART_DRIVE_LENGTH));
      HatchPickUpButton.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_STATION, RobotMap.CART_REVERSE_SAFETY_LENGTH));
      LiftingUnitButtonCalibrate.whenPressed(new CLiftingUnitCalibrate());
    
      /** Remote control replacement follows here */
      HatchStation = new JoystickButton(JoystickHeightController, RobotMap.HEIGHT_HATCH_STATION_BUTTON_ID);
      HatchRocket2 = new JoystickButton(JoystickHeightController, RobotMap.HEIGHT_HATCH_MID_BUTTON_ID);
      HatchRocket3 = new JoystickButton(JoystickHeightController, RobotMap.HEIGHT_HATCH_TOP_BUTTON_ID);
      CargoGroundButton2 = new JoystickButton(JoystickHeightController, RobotMap.HEIGHT_CARGO_GROUND_BUTTON_ID);

      HatchStation.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_STATION, RobotMap.CART_REVERSE_SAFETY_LENGTH));
      HatchRocket2.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_MID, RobotMap.CART_REVERSE_SAFETY_LENGTH));
      HatchRocket3.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_TOP, RobotMap.CART_REVERSE_SAFETY_LENGTH));
      CargoGroundButton2.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_DEPOT, RobotMap.CART_DRIVE_LENGTH));

      CargoRocket1 = new Button() {
        @Override
        public boolean get() {
          return (JoystickHeightController.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 180);
        }
       };
       CargoRocket1.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_BOTTOM, RobotMap.CART_DRIVE_LENGTH));

       CargoRocket2 = new Button() {
        @Override
        public boolean get() {
          return (JoystickHeightController.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 90);
        }
       };
       CargoRocket2.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_MID, RobotMap.CART_DRIVE_LENGTH));
       CargoRocket3 = new Button() {
        @Override
        public boolean get() {
          return (JoystickHeightController.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 0);
        }
       };
       CargoRocket3.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_TOP, RobotMap.CART_DRIVE_LENGTH));
       CargoStation = new Button() {
        @Override
        public boolean get() {
          return (JoystickHeightController.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 270);
        }
       };
       CargoStation.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_STATION, RobotMap.CART_REVERSE_SAFETY_LENGTH));


      }

  }
}
