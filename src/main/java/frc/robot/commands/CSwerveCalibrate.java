/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CSwerveCalibrate extends Command {
  public CSwerveCalibrate() {
    requires(Robot.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    for (IFridolinsMotors motor : Motors.swerveDriveMotors) {
			motor.setVelocity(0.5);
		}
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean allMotorsReverseLimitSwitchClosed = true;
		for (IFridolinsMotors motor : Motors.swerveDriveMotors) {
			allMotorsReverseLimitSwitchClosed &= motor.isReverseLimitSwitchActive();
		}
    return allMotorsReverseLimitSwitchClosed;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
			for (IFridolinsMotors motor : Motors.swerveDriveMotors) {
			motor.setVelocity(RobotMap.STOP_SPEED);
			motor.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, false);
			}
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
