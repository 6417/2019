/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SCargoGripper extends Subsystem {

  static ShuffleboardLayout cargoList = Robot.shuffleSubsystems.getLayout("Cargo Gripper", BuiltInLayouts.kList).withSize(1, 3).withPosition(4, 0);
  static NetworkTableEntry pullSpeed = cargoList.add("Pull", -RobotMap.CARGO_GRIPPER_SPEED).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Max", 0)).getEntry();
  static NetworkTableEntry pushSpeed = cargoList.add("Push", RobotMap.CARGO_GRIPPER_SPEED).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).getEntry();

  public SCargoGripper() {
  }

  @Override
  public void initDefaultCommand() {
  }

  public void push() {
    Motors.cargoGripperMaster.set(pushSpeed.getDouble(RobotMap.CARGO_GRIPPER_SPEED));
  }

  public void pull() {
    Motors.cargoGripperMaster.set(pullSpeed.getDouble(RobotMap.CARGO_GRIPPER_SPEED));
  }

  public void stop() {
    Motors.cargoGripperMaster.set(RobotMap.STOP_SPEED);
  }

  public boolean isLimitSwitchPressed() {
    return !Motors.cargoGripperMaster.isReverseLimitSwitchActive();
  }

}