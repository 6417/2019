/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import ch.fridolinsrobotik.drivesystems.swerve.commands.SwerveSteeringPutStraight;
import ch.fridolinsrobotik.drivesystems.swerve.commands.SwerveSteeringZeroingBySwitch;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CSwerveCalibrate extends CommandGroup {
  public CSwerveCalibrate() {
    addSequential(new SwerveSteeringZeroingBySwitch());
    addSequential(new SwerveSteeringPutStraight());
  }
}
