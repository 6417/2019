/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.liftingunit;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CLiftingUnitDriveToLimit extends Command {

  private double value;
  private boolean revLimitSwitchWasPressed;

  public CLiftingUnitDriveToLimit(double value) {
    requires(Robot.liftingUnit);
    this.value = value;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.liftingUnit.enableAutonomous(false);
    revLimitSwitchWasPressed =!Motors.liftMaster.getSensorCollection().isRevLimitSwitchClosed();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(revLimitSwitchWasPressed) {  
      if(!Motors.liftMaster.getSensorCollection().isRevLimitSwitchClosed()) {
        Robot.liftingUnit.drive(value); 
      } else {
        Robot.liftingUnit.drive((-value) / 4);
      } 
    } else {
      Robot.liftingUnit.drive(value);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.liftingUnit.isZeroed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.liftingUnit.stopMotor();
    Robot.liftingUnit.setTargetPosition(Robot.liftingUnit.getPosition());
    Robot.liftingUnit.enableAutonomous(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
