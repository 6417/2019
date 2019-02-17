/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Motors;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SHatchGripper extends Subsystem {

  public SHatchGripper() {
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void hatchGripperExtend() {
    Motors.hatchGripperMotor.setPercent(RobotMap.HATCH_GRIPPER_SPEED);
  }

  public static void hatchGripperRetract() {
    Motors.hatchGripperMotor.setPercent(-RobotMap.HATCH_GRIPPER_SPEED);
  }

  public static void hatchGripperStop() {
    Motors.hatchGripperMotor.setPercent(RobotMap.STOP_SPEED);
  }

}
