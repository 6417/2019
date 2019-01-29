/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import ch.fridolinsrobotik.motorcontrollers.FridolinsIdleModeType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;

/**
 * Add your docs here.
 */
public class Motors {

    //Create Motors
    public static IFridolinsMotors cargoGripperMotorRight;
    public static IFridolinsMotors cargoGripperMotorLeft;

    public static IFridolinsMotors cargoGripperMaster;

    public static IFridolinsMotors hatchGripperMotor;

    public static void initialize() {

        if(RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            //Initialize Motors
            cargoGripperMotorRight = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_RIGHT_ID);
            cargoGripperMotorLeft = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_LEFT_ID);

            cargoGripperMaster = cargoGripperMotorLeft;

            //Set Mode and Limit Switches
            cargoGripperMotorLeft.setIdleMode(FridolinsIdleModeType.kBrake); 
            cargoGripperMotorRight.setIdleMode(FridolinsIdleModeType.kBrake);

            cargoGripperMotorLeft.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
            cargoGripperMotorRight.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);

            cargoGripperMotorLeft.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
            cargoGripperMotorRight.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
        }

        if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            //Initialize Motors
            hatchGripperMotor = new FridolinsTalonSRX(RobotMap.HATCH_GRIPPER_MOTOR_ID);

            //Set Mode and Limit Switches
            hatchGripperMotor.setIdleMode(FridolinsIdleModeType.kBrake);

            hatchGripperMotor.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);

            hatchGripperMotor.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
        }

    }

}
