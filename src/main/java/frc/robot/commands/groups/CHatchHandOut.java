/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.cart.CCartSetPosition;
import frc.robot.commands.gripper.hatch.CHatchGripperRetract;

public class CHatchHandOut extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CHatchHandOut() {
    addSequential(new CHatchGripperRetract());
    addSequential(new CCartSetPosition(RobotMap.CART_DRIVE_LENGTH_HATCH_MM));
  }
}
