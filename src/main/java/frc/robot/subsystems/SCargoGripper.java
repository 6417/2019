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
public class SCargoGripper extends Subsystem {

  public SCargoGripper() {
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void cargoGripperPush() {
    Motors.cargoGripperMaster.setVelocity(RobotMap.CARGO_GRIPPER_SPEED);
  }

  public static void cargoGripperPull() {
    Motors.cargoGripperMaster.setVelocity(-RobotMap.CARGO_GRIPPER_SPEED);
  }

  public static void cargoGripperStop() {
    Motors.cargoGripperMaster.setVelocity(RobotMap.STOP_SPEED);
  }

}