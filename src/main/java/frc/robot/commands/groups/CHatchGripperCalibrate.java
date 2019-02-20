
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.gripper.hatch.CHatchGripperRetract;
import frc.robot.commands.gripper.hatch.CHatchGripperSeekLimitSwitch;

public class CHatchGripperCalibrate extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CHatchGripperCalibrate() {
    setName("Hatch Calibrate");
    addSequential(new CHatchGripperSeekLimitSwitch());
    addSequential(new CHatchGripperRetract());
  }
}
