/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.liftingunit;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CLiftingUnitSetHeight extends Command {
  int m_position;

  public CLiftingUnitSetHeight(int position) {
    requires(Robot.liftingUnit);
    m_position = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.liftingUnit.setTargetPosition(m_position);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.liftingUnit.drive();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.liftingUnit.isInRange(m_position - 100);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
