/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ch.fridolinsrobotik.motorcontrollers.FridolinsSparkMAX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SCargoGripper extends Subsystem {

  //Create Motors
  private static IFridolinsMotors motorRight;
  private static IFridolinsMotors motorLeft;

  //Create Time System for Debug Enoder Errors
  private static double timeStarted;
  private static double runTime;

  public SCargoGripper() {

    //Initialize Motors
    motorRight = new FridolinsSparkMAX(RobotMap.CARGO_GRIPPER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    motorLeft = new FridolinsSparkMAX(RobotMap.CARGO_GRIPPER_MOTOR_LEFT_ID, MotorType.kBrushless);
    
    //Initialize Time System
    timeStarted = Timer.getFPGATimestamp();
    runTime = 0;
  }

  @Override
  public void initDefaultCommand() {
  }

  public static void cargoGripperPush() {
    motorRight.setVelocity(0.1);
    motorLeft.setVelocity(-0.1);
  }

  public static boolean cargoGripperPull() {
    motorRight.setVelocity(-0.1);
    motorLeft.setVelocity(0.1); 
    return false;
  }

  public static void cargoGripperStop() {
    motorRight.setVelocity(0);
    motorLeft.setVelocity(0);
  }

  //Check if Encoders are working
  public static boolean healthy() {
    runTime = Timer.getFPGATimestamp() - timeStarted;

    //  // TODO add encoder ticks check
    //  if(runTime >= 1 /* && NO ENCODER TICKS SEEN */) {
    //   /* report error */
    //   return true;
    // } else if (false /* ENCODER TICKS REACHED POSITION */) {
    //   return true;
    // }
    return false;
  }

}