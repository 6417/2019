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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SHatchGripper extends Subsystem {

    private static IFridolinsMotors motorHatch;

  public SHatchGripper() {
    motorHatch = new FridolinsTalonSRX(RobotMap.HATCH_GRIPPER_MOTOR_ID);

    motorHatch.setIdleMode(FridolinsIdleModeType.kBrake);
    motorHatch.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
    motorHatch.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void hatchGripperExtend() {
    motorHatch.setVelocity(RobotMap.HATCH_GRIPPER_SPEED);
  }

  public static void hatchGripperRetract() {
    motorHatch.setVelocity(-RobotMap.HATCH_GRIPPER_SPEED);
  }

  public static void hatchGripperStop() {
    motorHatch.setVelocity(RobotMap.STOP_SPEED);
  }

}
