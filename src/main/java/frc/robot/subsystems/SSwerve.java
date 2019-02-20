/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motors;
import frc.robot.RobotMap;
import frc.robot.commands.drive.swerve.CSwerveDriveManual;

/**
 * Add your docs here.
 */
public class SSwerve extends Subsystem {

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CSwerveDriveManual());
	}
  
	ArrayList<Double> idealMotorRotateAngle = new ArrayList<Double>();

	public static final double _90DegreesInRad = Math.PI / 2;
	public static final double _180DegreesInRad = Math.PI;
	public static final double _360DegreesInRad = 2 * Math.PI;

	private double driveX;
	private double driveY;
	private double rotationMagnitude;
	private double gyro;
	private double multiplier;

	/**
	 * 
	 * @param driveMotors               Motor list in this order: front left, front
	 *                                  right, back right, back left
	 * @param steeringMotors            Motor list in this order: front left, front
	 *                                  right, back right, back left
	 * @param robotWidth                motors width distance width in cm
	 * @param robotLength               motors length distance in cm
	 * @param pulsesPerRotationSteering Pulses per rotation of the wheel when
	 *                                  rotating
	 * @param pulsesPerRotationDriving  Pulses per rotation of the wheel when
	 *                                  driving
	 */
	public SSwerve() {
			
			double alpha = Math.atan(RobotMap.WHEEL_DISTANCE_WIDTH / RobotMap.WHEEL_DISTANCE_LENGTH);

			/***
			 * front left and back right are equal, and front right and back left are equal
			 * in rectangles. front left and front right are mirrored in cartesian plane.
			 */

			// calculate angle for front left
			idealMotorRotateAngle.add(alpha);
			// calculate angle for front right
			idealMotorRotateAngle.add(- alpha);
			// copy front left to back right but let it face in the opposite direction
			idealMotorRotateAngle.add(alpha + _180DegreesInRad);// + _180DegreesInRad);
			// for back left it's same as front right but let it face in the opposite
			// direction
			idealMotorRotateAngle.add(- alpha + _180DegreesInRad);// + _180DegreesInRad);

			for(Double angle : idealMotorRotateAngle) {
					System.out.println(Math.toDegrees(angle));
			}
	}

	public void manualDrive(double driveX, double driveY, double rotationMagnitude, double gyro) {
		this.driveX = driveX;
		this.driveY = driveY;
		this.rotationMagnitude = rotationMagnitude;
		this.gyro = gyro;
	}

	public void setMultiplier(double multiplier) {
		this.multiplier = multiplier;
	}

	/**
	 * 
	 * @param driveX            power to translate sideways
	 * @param driveY            power to translate forward/backward
	 * @param rotationMagnitude rotation around the robot's own Z axis
	 * @param gyro              The current angle reading from the gyro in degrees
	 *                          around the Z axis. Use this to implement
	 *                          field-oriented controls
	 */
	public void driveCartesian() {

			for (int i = 0; i < idealMotorRotateAngle.size(); i++) {
					Double angle = idealMotorRotateAngle.get(i);
					Vector2d rotationVector = new Vector2d(rotationMagnitude * Math.cos(angle), rotationMagnitude * Math.sin(angle));
					Vector2d striveVector = new Vector2d(driveX, driveY);
					striveVector.rotate(gyro);
					Vector2d movingVector = new Vector2d(rotationVector.x + striveVector.x, rotationVector.y + striveVector.y);

					SmartDashboard.putNumber("rotation x " + i, rotationVector.x);
					SmartDashboard.putNumber("rotation y " + i, rotationVector.y);
					SmartDashboard.putNumber("strive x " + i, striveVector.x);
					SmartDashboard.putNumber("strive y " + i, striveVector.y);
					SmartDashboard.putNumber("moving x " + i, movingVector.x);
					SmartDashboard.putNumber("moving y " + i, movingVector.y);

					double magnitude = movingVector.magnitude();
					if(magnitude < RobotMap.DEADZONE_RANGE) {
						magnitude = 0;
					}
					magnitude = Algorithms.scale(magnitude, RobotMap.DEADZONE_RANGE, 1, 0, 1);
					
					drive(Motors.swerveAngleMotors.get(i), Motors.swerveDriveMotors.get(i), Math.atan2(movingVector.x, movingVector.y), magnitude, multiplier, i);
					// drive(Motors.swerveAngleMotors.get(i), Motors.swerveDriveMotors.get(i), OI.JoystickMainDriver.getDirectionRadians(), movingVector.magnitude(), i);
			}

	}

	private void drive(IFridolinsMotors iFridolinsMotors, IFridolinsMotors iFridolinsMotors2, double targetRadians, double magnitude, double multiplier, int debugI) {
			int steeringEncoderPulses = iFridolinsMotors.getEncoderTicks();
			double currentRadians = remainder(convertEncoderPulsesToRadians(steeringEncoderPulses, RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT), _360DegreesInRad);
			double deltaRadians = targetRadians - currentRadians + _360DegreesInRad;
			double steer = (deltaRadians + _90DegreesInRad) % _180DegreesInRad - _90DegreesInRad;
			double drive = -1 * (Math.floor((deltaRadians + _90DegreesInRad) / _180DegreesInRad) % 2 * 2 -1);

			int steeringEncoderGoal = steeringEncoderPulses + convertRadiansToEncoderPulses(steer, RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT);
			
			SmartDashboard.putNumber("steer " + debugI, Math.toDegrees(steer));
			SmartDashboard.putNumber("deltaPos " + debugI, Math.toDegrees(deltaRadians));
			SmartDashboard.putNumber("Current Radians" + debugI, Math.toDegrees(currentRadians));
			SmartDashboard.putNumber("Target Radians" + debugI, Math.toDegrees(targetRadians));
			SmartDashboard.putNumber("encoder pos " + debugI, steeringEncoderPulses);
			SmartDashboard.putNumber("SteeringEncoderGoal" + debugI, steeringEncoderGoal);
			SmartDashboard.putNumber("drive" + debugI, drive);
			SmartDashboard.putNumber("Drive Encoder " +debugI, iFridolinsMotors2.getEncoderTicks());
			SmartDashboard.putNumber("Magnitude " +debugI, magnitude);

			if(magnitude > 0) {
			iFridolinsMotors.setPosition(steeringEncoderGoal);
			}
			iFridolinsMotors2.setPercent(drive * magnitude * multiplier);
			
	}

	private double convertEncoderPulsesToRadians(int pulses, int pulsesPerRotation) {
			return (2 * Math.PI) / pulsesPerRotation * pulses;
	}

	private int convertRadiansToEncoderPulses(double radians, int pulsesPerRotation) {
			return (int) (pulsesPerRotation / (2 * Math.PI) * radians);
	}


	/**
	 * Modulo function, since Java implements % for negative values different. E.g., -182 % 180 => -2.
	 * This function calculates the remainder for negative inputs differently. E.g., -182 % 180 => -2 + 180 => 178
	 */
	public double remainder(double x, double m) {
			return ((x % m) + m) % m;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addBooleanProperty("HallSteeringFrontLeft", Motors.swerveAngleFrontLeft::isForwardLimitSwitchActive, null);
		builder.addBooleanProperty("HallSteeringFrontRight", Motors.swerveAngleFrontRight::isForwardLimitSwitchActive, null);
		builder.addBooleanProperty("HallSteeringBackLeft", Motors.swerveAngleBackLeft::isForwardLimitSwitchActive, null);
		builder.addBooleanProperty("HallSteeringBackRight", Motors.swerveAngleBackRight::isForwardLimitSwitchActive, null);
	}

}
