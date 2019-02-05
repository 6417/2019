/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import ch.fridolinsrobotik.drivesystems.fieldoriented.FieldOrientedDrive;
import ch.fridolinsrobotik.drivesystems.swerve.SwerveCalculation;
import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.utilities.Deadzone;
import ch.fridolinsrobotik.utilities.JoystickOptimizer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Motors;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SSwerve extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  //Variables
  private double joystickY, joystickX, joystickZ, joystickAngle, joystickPower, resultingAngle = 0;

  private int[] encoderDriveCount, encoderSteerCount = new int[4];

  private double[] wheelDirection, rotationDistance, currentWheelAngle, swerveDrivePower, swerveSteerAngle, talonSteerOutput, talonDriveOutput = new double[4];
  
  public static boolean[] rotationDirection = new boolean[4];

  private static final JoystickButton buttonEncoderSetZero = new JoystickButton(OI.JoystickMainDriver, 6);
	private static final JoystickButton buttonFieldAngleSetZero = new JoystickButton(OI.JoystickMainDriver, 5);
	private static final JoystickButton buttonCalibrateFrontLeft = new JoystickButton(OI.JoystickMainDriver, 4);
	private static final JoystickButton buttonCalibrateFrontRight = new JoystickButton(OI.JoystickMainDriver, 2);
	private static final JoystickButton buttonCalibrateBackLeft = new JoystickButton(OI.JoystickMainDriver, 1);
  private static final JoystickButton buttonCalibrateBackRight = new JoystickButton(OI.JoystickMainDriver, 3);
  
  private boolean buttonEncoderSetZeroState, buttonNavXSetZeroState, buttonCalibrateFrontLeftState, buttonCalibrateFrontRightState, buttonCalibrateBackLeftState, buttonCalibrateBackRightState = false;

  private SwerveCalculation swerveCalculation;

  public SSwerve() {
    swerveCalculation = new SwerveCalculation(RobotMap.WHEEL_DISTANCE_LENGTH, RobotMap.WHEEL_DISTANCE_WIDTH);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void driveCartesian(double x, double y, double rotation, double gyro) {
    joystickX = Deadzone.getAxis(OI.JoystickMainDriver.getX(), RobotMap.DEADZONE_RANGE);
		joystickY = Deadzone.getAxis(-OI.JoystickMainDriver.getY(), RobotMap.DEADZONE_RANGE);
		joystickZ = Deadzone.getAxis(-OI.JoystickMainDriver.getZ(), RobotMap.DEADZONE_RANGE);

    joystickAngle = JoystickOptimizer.getJoystickAngle(OI.JoystickMainDriver.getX(), -OI.JoystickMainDriver.getY(), RobotMap.DEADZONE_RANGE);
    joystickPower = JoystickOptimizer.getDrivePower(OI.JoystickMainDriver.getX(), -OI.JoystickMainDriver.getY(), RobotMap.DEADZONE_RANGE);
    
    resultingAngle = FieldOrientedDrive.resultingAngle(joystickAngle, gyro);

    encoderSteerCount[0] = Motors.swerveAngleFrontLeft.getEncoderTicks();
		encoderSteerCount[1] = -Motors.swerveAngleFrontRight.getEncoderTicks();
		encoderSteerCount[2] = -Motors.swerveAngleBackLeft.getEncoderTicks();
    encoderSteerCount[3] = -Motors.swerveAngleBackRight.getEncoderTicks();
		encoderDriveCount[0] = Motors.swerveDriveFrontLeft.getEncoderTicks();
		encoderDriveCount[1] = Motors.swerveDriveFrontRight.getEncoderTicks();
		encoderDriveCount[2] = Motors.swerveDriveBackLeft.getEncoderTicks();
    encoderDriveCount[3] = Motors.swerveDriveBackRight.getEncoderTicks();

    buttonEncoderSetZeroState = buttonEncoderSetZero.get();
		buttonNavXSetZeroState = buttonFieldAngleSetZero.get();
		buttonCalibrateFrontLeftState = buttonCalibrateFrontLeft.get();
		buttonCalibrateFrontRightState = buttonCalibrateFrontRight.get();
		buttonCalibrateBackLeftState = buttonCalibrateBackLeft.get();
		buttonCalibrateBackRightState = buttonCalibrateBackRight.get();
    
    swerveCalculation.calculateValues(resultingAngle, joystickPower, joystickZ);
		swerveDrivePower = swerveCalculation.getDrivePower();
		swerveSteerAngle = swerveCalculation.getSteerAngle();
		
		if(buttonCalibrateFrontLeftState == true) {
			
			Motors.swerveAngleFrontLeft.setVelocity(OI.JoystickMainDriver.getX());
			
		}else if(buttonCalibrateFrontRightState == true) {
			
			Motors.swerveAngleFrontRight.setVelocity(OI.JoystickMainDriver.getX());
			
		}else if(buttonCalibrateBackLeftState == true) {
			
			Motors.swerveAngleBackLeft.setVelocity(OI.JoystickMainDriver.getX());
			
		}else if(buttonCalibrateBackRightState == true) {
			
			Motors.swerveAngleBackRight.setVelocity(OI.JoystickMainDriver.getX());
			
		}else {
						
			currentWheelAngle[0] = swerveCalculation.getWheelAngleDegrees(encoderSteerCount[0], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT);
			currentWheelAngle[1] = swerveCalculation.getWheelAngleDegrees(encoderSteerCount[1], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT);
			currentWheelAngle[2] = swerveCalculation.getWheelAngleDegrees(encoderSteerCount[2], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT);
			currentWheelAngle[3] = swerveCalculation.getWheelAngleDegrees(encoderSteerCount[3], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT);
	
			for(int i = 0; i < 4; i++) {
				swerveCalculation.calculateFastestWayToAngle(currentWheelAngle[i], swerveSteerAngle[i]);
				wheelDirection[i] = swerveCalculation.getWheelDirection();
				rotationDirection[i] = swerveCalculation.getRotationDirection();
				rotationDistance[i] = swerveCalculation.getRotationDistance();
			}
			
			talonSteerOutput[0] = swerveCalculation.getTalonSteerOutput(encoderSteerCount[0], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT, rotationDistance[0], rotationDirection[0]);
			talonSteerOutput[1] = swerveCalculation.getTalonSteerOutput(encoderSteerCount[1], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT, rotationDistance[1], rotationDirection[1]);
			talonSteerOutput[2] = swerveCalculation.getTalonSteerOutput(encoderSteerCount[2], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT, rotationDistance[2], rotationDirection[2]);
			talonSteerOutput[3] = swerveCalculation.getTalonSteerOutput(encoderSteerCount[3], RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT, rotationDistance[3], rotationDirection[3]);
	
			for(int i = 0; i < 4; i++) {
				talonDriveOutput[i] = swerveDrivePower[i] * wheelDirection[i] * RobotMap.DRIVE_SPEED_MULITPLIER;
			}
				
			Motors.swerveAngleFrontLeft.setVelocity(talonSteerOutput[0]);
			Motors.swerveAngleFrontRight.setVelocity(talonSteerOutput[1]);
      Motors.swerveAngleBackLeft.setVelocity(talonSteerOutput[2]);
			Motors.swerveAngleBackRight.setVelocity(talonSteerOutput[3]);
			
      Motors.swerveDriveFrontLeft.setVelocity(-talonDriveOutput[0]);
			Motors.swerveDriveFrontRight.setVelocity(talonDriveOutput[1]);
			Motors.swerveDriveBackLeft.setVelocity(-talonDriveOutput[2]);
			Motors.swerveDriveBackRight.setVelocity(talonDriveOutput[3]);
			
		}
	
		if(buttonEncoderSetZeroState == true) {
      Motors.swerveDriveFrontLeft.setSensorPosition(0);
      Motors.swerveDriveFrontRight.setSensorPosition(0);
      Motors.swerveDriveBackLeft.setSensorPosition(0);
      Motors.swerveDriveBackRight.setSensorPosition(0);
      Motors.swerveAngleFrontLeft.setSensorPosition(0);
      Motors.swerveAngleFrontRight.setSensorPosition(0);
      Motors.swerveAngleBackLeft.setSensorPosition(0);
      Motors.swerveAngleBackRight.setSensorPosition(0);
		}
		
		if(buttonNavXSetZeroState == true) {
			Robot.ahrs.reset();		
		}

		
	}

  public void driveCartesian(double x, double y, double rotation) {
    driveCartesian(x, y, rotation, 0.0);
	}

}
