/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.cart.CCartDriveToLimit;
import frc.robot.commands.cart.CCartSetPosition;
import frc.robot.commands.liftingunit.CLiftingUnitDriveToLimit;
import frc.robot.commands.liftingunit.CLiftingUnitSetHeight;

public class CLiftingUnitCalibrate extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CLiftingUnitCalibrate() {
    addSequential(new CLiftingUnitDriveToLimit(0.5));
    addSequential(new CLiftingUnitSetHeight(RobotMap.LIFTING_UNIT_HEIGHT_START));
    addSequential(new CCartDriveToLimit(0.3));
    addSequential(new CCartSetPosition(RobotMap.CART_CENTER_POINT));
  }
}
