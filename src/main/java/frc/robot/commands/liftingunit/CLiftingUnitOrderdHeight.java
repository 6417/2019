/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.liftingunit;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.groups.CAutonomousDeliver;

public class CLiftingUnitOrderdHeight extends Command {
  public CLiftingUnitOrderdHeight() {
  }

  static NetworkTableEntry ballStation = Robot.raspberry.getEntry("BallStation");;
  static NetworkTableEntry hatchStation = Robot.raspberry.getEntry("HatchStation");;
  static NetworkTableEntry startPosition = Robot.raspberry.getEntry("StartPosition");;
  static NetworkTableEntry ballDepot = Robot.raspberry.getEntry("BallRamp");;
  static NetworkTableEntry ballCargoship = Robot.raspberry.getEntry("CargoOben");;
  static NetworkTableEntry ballUnten = Robot.raspberry.getEntry("BallUnten");;
  static NetworkTableEntry hatchOben = Robot.raspberry.getEntry("HatchOben");;
  static NetworkTableEntry ballOben = Robot.raspberry.getEntry("BallOben");;
  static NetworkTableEntry hatchMitte = Robot.raspberry.getEntry("HatchMitte");;
  static NetworkTableEntry ballMitte = Robot.raspberry.getEntry("BallMitte");;
  static NetworkTableEntry hatchUnten = Robot.raspberry.getEntry("HatchUnten");;
  static NetworkTableEntry hatchCargoship = Robot.raspberry.getEntry("CargoUnten");;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.cart.enableAutonomous(true);
    Robot.liftingUnit.enableAutonomous(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (hatchOben.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_TOP, RobotMap.CART_REVERSE_SAFETY_LENGHT).start();
      // Robot.liftingUnit.setTargetPosition(26150);
    }
    if (hatchMitte.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_MID, RobotMap.CART_REVERSE_SAFETY_LENGHT).start();
      // Robot.liftingUnit.setTargetPosition(13740);
    }
    if (hatchUnten.getBoolean(false) || hatchStation.getBoolean(false) || hatchCargoship.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_STATION, RobotMap.CART_REVERSE_SAFETY_LENGHT).start();
      // Robot.liftingUnit.setTargetPosition(2050);
    }
    if (ballOben.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_TOP, RobotMap.CART_DRIVE_LENGTH).start();
    }
    if (ballMitte.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_MID, RobotMap.CART_DRIVE_LENGTH).start();
    }
    if (ballUnten.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_BOTTOM, RobotMap.CART_DRIVE_LENGTH).start();
    }
    if (ballCargoship.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_SHIP, RobotMap.CART_DRIVE_LENGTH).start();
    }
    if (ballCargoship.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_STATION, RobotMap.CART_DRIVE_LENGTH).start();
    }
    if (ballDepot.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_DEPOT, RobotMap.CART_DRIVE_LENGTH).start();
    }

    if (startPosition.getBoolean(false)) {
      new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_DEPOT, RobotMap.CART_CENTER_POINT).start();
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
    System.out.println("Command ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("command interrupted");
  }
}
