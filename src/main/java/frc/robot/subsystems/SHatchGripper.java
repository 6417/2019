/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.groups.CHatchGripperCalibrate;

/**
 * Add your docs here.
 */
public class SHatchGripper extends Subsystem {

  static ShuffleboardLayout hatchList = Robot.shuffleSubsystems.getLayout("Hatch Gripper", BuiltInLayouts.kList).withSize(1, 3).withPosition(4, 0);
  static NetworkTableEntry hatchMode = hatchList.add("Mode", "").getEntry();
  static NetworkTableEntry hatchTopSwitch = hatchList.add("Top Switch", false).getEntry();
  static NetworkTableEntry hatchLeftSwitch = hatchList.add("Left Switch", false).getEntry();
  static NetworkTableEntry hatchRightSwitch = hatchList.add("Right Switch", false).getEntry();
  static DigitalInput topSwitch = new DigitalInput(RobotMap.HATCH_GRIPPER_DIO_TOP);
  static DigitalInput rightSwitch = new DigitalInput(RobotMap.HATCH_GRIPPER_DIO_RIGHT);
  static DigitalInput leftSwitch = new DigitalInput(RobotMap.HATCH_GRIPPER_DIO_LEFT);

  boolean isHomed = false;

  public SHatchGripper() {
  }

  @Override
  public void initDefaultCommand() {
    hatchList.add("Hatch Calibrate", new CHatchGripperCalibrate());
  }

  public void hatchGripperExtend() {
    // Motors.hatchGripperMotor.set(RobotMap.HATCH_GRIPPER_SPEED);
    Motors.hatchGripperMotor.set(ControlMode.MotionMagic, RobotMap.HATCH_DRIVE_EXTENDED);
    // Motors.hatchGripperMotor.set(-OI.JoystickMainDriver.getY());
  }
  public void hatchGripperRetract() {
    // Motors.hatchGripperMotor.set(-RobotMap.HATCH_GRIPPER_SPEED);
    Motors.hatchGripperMotor.set(ControlMode.MotionMagic, RobotMap.HATCH_DRIVE_RETRACTED);
    // Motors.hatchGripperMotor.set(-OI.JoystickMainDriver.getY());
  }

  public void hatchGripperStop() {
    Motors.hatchGripperMotor.setPercent(RobotMap.STOP_SPEED);
  }

  public void homing() {
    Motors.hatchGripperMotor.setPercent(RobotMap.HATCH_CALIBRATE_SPEED);
  }

  public void automaticResetHatchEncoder() {
    if (getReverseLimit()) {
      Motors.hatchGripperMotor.setSelectedSensorPosition(RobotMap.HATCH_DRIVE_RETRACTED);
      isHomed = true;
      // hatchGripperStop();
    } else if (getForwardLimit()) {
      Motors.hatchGripperMotor.setSelectedSensorPosition(RobotMap.HATCH_DRIVE_EXTENDED);
      isHomed = true;
      // hatchGripperStop();
    }
  }


  @Override
  public void periodic() {
    hatchMode.setString(getForwardLimit() ? "Extended" : "Retracted");
    hatchTopSwitch.setBoolean(getTopSwitch());
    hatchLeftSwitch.setBoolean(getLeftSwitch());
    hatchRightSwitch.setBoolean(getRightSwitch());
  }

  public boolean isHomed() {
    return isHomed;
  }

  public boolean isExtended() {
    return Motors.hatchGripperMotor.getSelectedSensorPosition() >= RobotMap.HATCH_DRIVE_EXTENDED - RobotMap.HATCH_DRIVE_RETRACTED;
  }

  public boolean getForwardLimit() {
    return Motors.hatchGripperMotor.isForwardLimitSwitchActive();
  }

  public boolean getReverseLimit() {
    return Motors.hatchGripperMotor.isReverseLimitSwitchActive();
  }

  public boolean getTopSwitch() {
    return topSwitch.get();
  }

  public boolean getLeftSwitch() {
    return leftSwitch.get();
  }

  public boolean getRightSwitch() {
    return rightSwitch.get();
  }

  public int getButtons() {
    ArrayList<Boolean> buttons = new ArrayList<>();
    buttons.add(getTopSwitch());
    buttons.add(getLeftSwitch());
    buttons.add(getRightSwitch());

    int buttonsPressed = 0;
    for(Boolean button : buttons) {
      if(button.booleanValue()) {
        buttonsPressed++;
      }
    }
    return buttonsPressed;
  }

}
