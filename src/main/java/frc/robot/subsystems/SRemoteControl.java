/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.groups.CAutonomousDeliver;

/**
 * Add your docs here.
 */
public class SRemoteControl extends Subsystem {

  static Button ballStation = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("BallStation").getBoolean(false);
    }
  };

  static Button hatchStation = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("HatchStation").getBoolean(false);
    }
  };

  static Button startPosition = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("StartPosition").getBoolean(false);
    }
  };

  static Button ballDepot = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("BallRamp").getBoolean(false);
    }
  };

  static Button ballCargoship = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("CargoOben").getBoolean(false);
    }
  };

  static Button ballUnten = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("BallUnten").getBoolean(false);
    }
  };

  static Button hatchOben = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("HatchOben").getBoolean(false);
    }
  };

  static Button ballOben = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("BallOben").getBoolean(false);
    }
  };

  static Button hatchMitte = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("HatchMitte").getBoolean(false);
    }
  };

  static Button ballMitte = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("BallMitte").getBoolean(false);
    }
  };

  static Button hatchUnten = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("HatchUnten").getBoolean(false);
    }
  };

  static Button hatchCargoship = new Button() {

    @Override
    public boolean get() {
      return Robot.raspberry.getEntry("CargoUnten").getBoolean(false);
    }
  };

  public SRemoteControl() {
    hatchOben.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_TOP, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    hatchMitte.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_MID, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    hatchUnten.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_BOTTOM, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    hatchStation.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_STATION, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    hatchCargoship.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_HATCH_BOTTOM, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    ballOben.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_TOP, RobotMap.CART_DRIVE_LENGTH));
    ballMitte.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_MID, RobotMap.CART_DRIVE_LENGTH));
    ballUnten.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_BOTTOM, RobotMap.CART_DRIVE_LENGTH));
    ballCargoship.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_SHIP, RobotMap.CART_FORWARD_SAFETY_LENGTH));
    ballDepot.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_DEPOT, RobotMap.CART_DRIVE_LENGTH));
    ballStation.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_CARGO_STATION, RobotMap.CART_REVERSE_SAFETY_LENGTH));
    startPosition.whenPressed(new CAutonomousDeliver(RobotMap.LIFTING_UNIT_HEIGHT_START, RobotMap.CART_CENTER_POINT));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
