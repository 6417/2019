/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve.commands;

import ch.fridolinsrobotik.motorcontrollers.FridolinsIdleModeType;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Motors;
import frc.robot.Robot;

public class SwerveSteeringPutStraight extends Command {
  private static final int targetEncoderTicks = (int)(196608.0/360.0*105);
  private static final int maximumError = 500;

  public SwerveSteeringPutStraight() {
    requires(Robot.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    for(IFridolinsMotors motor : Motors.swerveAngleMotors) {
      motor.setIdleMode(FridolinsIdleModeType.kBrake);
      motor.setPosition(targetEncoderTicks);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean allWheelAngleCorrect = true;
    for(IFridolinsMotors motor : Motors.swerveAngleMotors) {
      allWheelAngleCorrect &= (motor.getClosedLoopError() < maximumError); 
      }
    return allWheelAngleCorrect;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    for(IFridolinsMotors motor : Motors.swerveAngleMotors) {
      motor.setSelectedSensorPosition(0);
      motor.setPercent(0);
      motor.setIdleMode(FridolinsIdleModeType.kCoast);
    }
    Robot.swerveDrive.homed(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
