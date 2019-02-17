/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.subsystems.SHatchGripper;

public class CHatchGripperRetract extends Command {
  public CHatchGripperRetract() {
    requires(Robot.hatchGripper);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // SHatchGripper.hatchGripperRetract();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SHatchGripper.hatchGripperRetract();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Motors.hatchGripperMotor.isReverseLimitSwitchActive();
    // return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SHatchGripper.hatchGripperStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
