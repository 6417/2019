/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import ch.fridolinsrobotik.drivesystems.swerve.commands.SwerveSteeringPutStraight;
import ch.fridolinsrobotik.drivesystems.swerve.commands.SwerveSteeringZeroingBySwitchWithPositionControl;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class CSwerveCalibrate extends CommandGroup {
  public CSwerveCalibrate() {
    // addSequential(new InstantCommand(new Runnable(){
    
    //   @Override
    //   public void run() {
    //     Robot.swerveDrive.homed(false);
    //   }
    // }));
    addSequential(new InstantCommand(() -> Robot.swerveDrive.homed(false)));
    addSequential(new SwerveSteeringZeroingBySwitchWithPositionControl(500));
    addSequential(new SwerveSteeringPutStraight());
  }
}
