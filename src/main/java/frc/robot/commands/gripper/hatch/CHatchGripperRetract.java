/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.gripper.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CHatchGripperRetract extends Command {
  public CHatchGripperRetract() {
    requires(Robot.hatchGripper);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchGripper.automaticResetHatchEncoder();
    Robot.hatchGripper.hatchGripperRetract();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Motors.hatchGripperMotor.isReverseLimitSwitchActive();
    if(Robot.hatchGripper.getButtons() <= 2) {
      return true;
    }
    return !Robot.hatchGripper.isHomed() || Robot.hatchGripper.getReverseLimit();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.hatchGripper.hatchGripperStop();
    // Motors.hatchGripperMotor.setSensorPosition(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
