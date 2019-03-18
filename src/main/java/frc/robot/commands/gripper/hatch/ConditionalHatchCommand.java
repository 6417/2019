/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.gripper.hatch;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ConditionalHatchCommand extends ConditionalCommand {

  public ConditionalHatchCommand(Command onTrue) {
        super(onTrue);
    }

    @Override
      protected boolean condition() {
        if(Robot.cart.getPosition() <= RobotMap.CART_REVERSE_SAFETY_LENGTH + RobotMap.CART_POSITION_ZONE) {
          return true;
        }
        return false;
      }

}
