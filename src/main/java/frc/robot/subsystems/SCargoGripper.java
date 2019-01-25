/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import ch.fridolinsrobotik.motorcontrollers.FridolinsIdleModeType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.watchdog.EncoderWatchDog;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SCargoGripper extends Subsystem {

  //Create Motors
  private static IFridolinsMotors motorRight;
  private static IFridolinsMotors motorLeft;

  private static double motorRightEncoderTicks;
  private static double motorLeftEncoderTicks;

  private static EncoderWatchDog motorRightEncoderChecker;
  private static EncoderWatchDog motorLeftEncoderChecker;

  public SCargoGripper() {

    //Initialize Motors
    motorRight = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_RIGHT_ID);
    motorLeft = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_LEFT_ID);

    //Set Mode and Limit Switches
    motorRight.setIdleMode(FridolinsIdleModeType.kBrake); 
    motorLeft.setIdleMode(FridolinsIdleModeType.kBrake);
    motorRight.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
    motorLeft.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
    motorRight.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
    motorLeft.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
  
    motorRightEncoderChecker = new EncoderWatchDog(1, 100);
    motorLeftEncoderChecker = new EncoderWatchDog(1, 100);
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void cargoGripperPush() {
    motorRight.setVelocity(RobotMap.CARGO_GRIPPER_SPEED);
    motorLeft.setVelocity(-RobotMap.CARGO_GRIPPER_SPEED);
  }

  public static boolean cargoGripperPull() {

    motorRight.setVelocity(-RobotMap.CARGO_GRIPPER_SPEED);
    motorLeft.setVelocity(RobotMap.CARGO_GRIPPER_SPEED);
    return false;
  }

  public static void cargoGripperStop() {
    motorRight.setVelocity(RobotMap.STOP_SPEED);
    motorLeft.setVelocity(RobotMap.STOP_SPEED);

    motorRightEncoderChecker.deactivate();
    motorLeftEncoderChecker.deactivate();
  }

  //Check if the Encoders are working in another Function who can be used in all Subsystems.
  public static boolean isMotorLefthealthy() {
    motorLeftEncoderTicks = motorLeft.getEncoderTicks();
    return motorLeftEncoderChecker.healthy(motorLeftEncoderTicks);
  }

  public static boolean isMotorRighthealthy() {
    motorRightEncoderTicks = motorRight.getEncoderTicks();
    return motorRightEncoderChecker.healthy(motorRightEncoderTicks);
  }

}