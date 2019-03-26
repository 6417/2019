/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.commands.drive.swerve.CSwerveDriveManual;

/**
 * Add your docs here.
 */
public class SSwerve extends Subsystem {

	private double driveX;
	private double driveY;
	private double rotationMagnitude;
	private double gyro;

	ShuffleboardLayout swerveSettings = Robot.shuffleSettings.getLayout("Swerve", BuiltInLayouts.kList).withPosition(2, 0).withSize(2,2);
	NetworkTableEntry homed = swerveSettings.add("Homed", false).getEntry();

	public SSwerve() {
		super();
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CSwerveDriveManual());
	}

	public void manualDrive(double driveX, double driveY, double rotationMagnitude, double gyro) {
		this.driveX = driveX;
		this.driveY = driveY;
		this.rotationMagnitude = rotationMagnitude;
		this.gyro = gyro;
	}

	public void setMultiplier(double multiplier) {
		Robot.swerve.setSpeedFactor(multiplier);
	}

	public void driveCartesian() {
		Robot.swerve.driveCartesian(driveX, driveY, rotationMagnitude, gyro);
	}

	public boolean isHomed() {
		return homed.getBoolean(false);
	}

	public void homed(boolean successful) {
		homed.setBoolean(successful);
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
