/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.StringJoiner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * {@inheritDoc} A Swerve module using for both steering and driving TalonSRX'
 */
public class SwerveModuleTalonSRX extends SwerveModule {
    private static int instances;
    WPI_TalonSRX drivingMotor;
    WPI_TalonSRX steeringMotor;

    public SwerveModuleTalonSRX(Vector2d mountingPosition, WPI_TalonSRX steeringMotor, int steeringPulsesPerRotation, WPI_TalonSRX drivingMotor, int drivingPulsesPerRotation) {
        super(mountingPosition, steeringPulsesPerRotation, drivingPulsesPerRotation);
        verify(drivingMotor, steeringMotor);
        this.drivingMotor = drivingMotor; this.steeringMotor = steeringMotor;
        instances++;
        setName("SwerveModuleTalonSRX", instances);
    }

    /**
     * Verifies that all motors are nonnull, throwing a NullPointerException if any
     * of them are. The exception's error message will specify all null motors, e.g.
     * {@code
     * NullPointerException("drivingMotor, steeringMotor")}, to give as much
     * information as possible to the programmer.
     *
     * @throws NullPointerException if any of the given motors are null
     */
    private void verify(WPI_TalonSRX drivingMotor, WPI_TalonSRX steeringMotor) {
        if (drivingMotor != null && steeringMotor != null) {
            return;
        }
        StringJoiner joiner = new StringJoiner(", ");
        if (drivingMotor == null) {
            joiner.add("drivingMotor");
        }
        if (steeringMotor == null) {
            joiner.add("steeringMotor");
        }
        throw new NullPointerException(joiner.toString());
    }

    @Override
    public void executeSwerveMovement() {
        steeringMotor.set(ControlMode.Position, getSteeringPosition());
        drivingMotor.set(ControlMode.PercentOutput, getDriveSpeedPercentage());
    }

    @Override
    public int getSteeringEncoderPulses() {
        return steeringMotor.getSelectedSensorPosition();
    }

    @Override
    public int getDriveEncoderPulses() {
        return drivingMotor.getSelectedSensorPosition();
    }

    @Override
    public boolean homingSensorActive() {
        return steeringMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    @Override
    public void setSteeringVelocity(int unitsPer100ms) {
        steeringMotor.set(ControlMode.Velocity, unitsPer100ms);
    }

    @Override
    public void stopMotors() {
        drivingMotor.stopMotor();
        steeringMotor.stopMotor();
    }

    @Override
    public void resetSteeringEncoder() {
        steeringMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void resetDrivingEncoder() {
        drivingMotor.setSelectedSensorPosition(0);
    }

}
