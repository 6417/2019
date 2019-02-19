/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CLiftingUnitSetHeight extends Command {
  public CLiftingUnitSetHeight() {
    requires(Robot.liftingUnit);
  }

  NetworkTableEntry ballStation;
  NetworkTableEntry hatchStation;
  NetworkTableEntry startPosition;
  NetworkTableEntry ballRamp;
  NetworkTableEntry cargoOben;
  NetworkTableEntry ballUnten;
  NetworkTableEntry hatchOben;
  NetworkTableEntry ballOben;
  NetworkTableEntry hatchMitte;
  NetworkTableEntry ballMitte;
  NetworkTableEntry hatchUnten;
  NetworkTableEntry cargoUnten;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ballStation = Robot.raspberry.getEntry("BallStation");
    hatchStation = Robot.raspberry.getEntry("HatchStation");
    startPosition = Robot.raspberry.getEntry("StartPosition");
    ballRamp = Robot.raspberry.getEntry("BallRamp");
    cargoOben = Robot.raspberry.getEntry("CargoOben");
    ballUnten = Robot.raspberry.getEntry("BallUnten");
    hatchOben = Robot.raspberry.getEntry("HatchOben");
    ballOben = Robot.raspberry.getEntry("BallOben");
    hatchMitte = Robot.raspberry.getEntry("HatchMitte");
    ballMitte = Robot.raspberry.getEntry("BallMitte");
    hatchUnten = Robot.raspberry.getEntry("HatchUnten");
    cargoUnten = Robot.raspberry.getEntry("CargoUnten");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(hatchOben.getBoolean(false) == true) {
      System.out.println("we got it up!!");
      Robot.liftingUnit.drive(7500);
    }
    if(hatchMitte.getBoolean(false) == true) {
      System.out.println("we got it mid!!");
      // Robot.liftingUnit.drive(8000);
    }
    if(hatchUnten.getBoolean(false) == true) {
      System.out.println("we got it down!!");
      // Robot.liftingUnit.drive(2000);
    }
    if(hatchStation.getBoolean(false) == true) {
      System.out.println("we got it down!!");
      // Robot.liftingUnit.drive(2000);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
