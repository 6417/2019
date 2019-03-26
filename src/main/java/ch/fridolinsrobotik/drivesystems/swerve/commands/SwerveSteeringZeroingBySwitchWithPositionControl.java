/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve.commands;

import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Motors;
import frc.robot.Robot;

public class SwerveSteeringZeroingBySwitchWithPositionControl extends Command {
  int encoderIncrement;
  int nextEncoderIncrement;
  int executeCounter;
  public SwerveSteeringZeroingBySwitchWithPositionControl(int encoderIncrement) {
    requires(Robot.swerveDrive);
    this.encoderIncrement = encoderIncrement;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    this.nextEncoderIncrement = this.encoderIncrement;
    executeCounter = 0;
    for (IFridolinsMotors steering : Motors.swerveAngleMotors) {
      steering.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
      steering.setPercent(0);
      steering.setSelectedSensorPosition(0);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    for (IFridolinsMotors steering : Motors.swerveAngleMotors) {
      steering.setPosition(nextEncoderIncrement);
    }
    nextEncoderIncrement += encoderIncrement;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean allLimitSwitchesHit = true;
    for (IFridolinsMotors steering : Motors.swerveAngleMotors) {
      allLimitSwitchesHit &= steering.isForwardLimitSwitchActive();
    }
    return allLimitSwitchesHit;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    for (IFridolinsMotors steering : Motors.swerveAngleMotors) {
      steering.setPercent(0);
      steering.setSensorPosition(0);
      steering.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kDisabled, false);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
