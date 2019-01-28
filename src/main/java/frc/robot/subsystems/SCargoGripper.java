/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import ch.fridolinsrobotik.watchdog.EncoderWatchDog;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Motors;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SCargoGripper extends Subsystem {

  private static double motorRightEncoderTicks;
  private static double motorLeftEncoderTicks;

  private static EncoderWatchDog motorRightEncoderChecker;
  private static EncoderWatchDog motorLeftEncoderChecker;

  public SCargoGripper() {
  
    motorRightEncoderChecker = new EncoderWatchDog("SCargoGripper Right", 1, 100);
    motorLeftEncoderChecker = new EncoderWatchDog("SCargoGripper Left", 1, 100);
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void cargoGripperPush() {
    Motors.cargoGripperMotorRight.setVelocity(RobotMap.CARGO_GRIPPER_SPEED);
    Motors.cargoGripperMotorLeft.setVelocity(RobotMap.CARGO_GRIPPER_SPEED);
  }

  public static boolean cargoGripperPull() {

    Motors.cargoGripperMotorRight.setVelocity(-RobotMap.CARGO_GRIPPER_SPEED);
    Motors.cargoGripperMotorLeft.setVelocity(-RobotMap.CARGO_GRIPPER_SPEED);
    return false;
  }

  public static void cargoGripperStop() {
    Motors.cargoGripperMotorRight.setVelocity(RobotMap.STOP_SPEED);
    Motors.cargoGripperMotorLeft.setVelocity(RobotMap.STOP_SPEED);

    motorRightEncoderChecker.deactivate();
    motorLeftEncoderChecker.deactivate();
  }

  //Check if the Encoders are working in another Function who can be used in all Subsystems.
  public static boolean isMotorLefthealthy() {
    motorLeftEncoderTicks = Motors.cargoGripperMotorLeft.getEncoderTicks();
    return motorLeftEncoderChecker.healthy(motorLeftEncoderTicks);
  }

  public static boolean isMotorRighthealthy() {
    motorRightEncoderTicks = Motors.cargoGripperMotorRight.getEncoderTicks();
    return motorRightEncoderChecker.healthy(motorRightEncoderTicks);
  }

}