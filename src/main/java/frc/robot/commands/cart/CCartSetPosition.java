/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cart;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CCartSetPosition extends Command {
  private int m_position;

  public CCartSetPosition(int position) {
    requires(Robot.cart);
    m_position = position;
  }
 

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.cart.setPosition(m_position);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.cart.drive();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.cart.isInRange(m_position);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.liftingUnit.setMaximumHeight(RobotMap.LIFTING_UNIT_DRIVE_LENGTH);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
