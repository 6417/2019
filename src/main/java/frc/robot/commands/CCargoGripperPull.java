/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SCargoGripper;

public class CCargoGripperPull extends Command {
  public CCargoGripperPull() {
    requires(Robot.cargoGripper);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SCargoGripper.cargoGripperPull();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
     return SCargoGripper.healthy();  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SCargoGripper.cargoGripperStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
