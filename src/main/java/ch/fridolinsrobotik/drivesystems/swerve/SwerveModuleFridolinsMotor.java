/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.StringJoiner;

import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * Add your docs here.
 */
public class SwerveModuleFridolinsMotor extends SwerveModule {
    private static int instances;
    IFridolinsMotors drivingMotor;
    IFridolinsMotors steeringMotor;

    public SwerveModuleFridolinsMotor(Vector2d mountingPosition, IFridolinsMotors steeringMotor,
            int steeringPulsesPerRotation, IFridolinsMotors drivingMotor, int drivingPulsesPerRotation) {
        super(mountingPosition, steeringPulsesPerRotation, drivingPulsesPerRotation);
        verify(drivingMotor, steeringMotor);
        this.drivingMotor = drivingMotor;
        this.steeringMotor = steeringMotor;
        addChild(drivingMotor);
        addChild(steeringMotor);
        instances++;
        setName("SwerveModuleFridolinsMotor", instances);
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
    private void verify(IFridolinsMotors drivingMotor, IFridolinsMotors steeringMotor) {
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
        steeringMotor.setPosition(getSteeringPosition());
        drivingMotor.setPercent(getDriveSpeedPercentage());
    }

    @Override
    public boolean homingSensorActive() {
        return steeringMotor.isForwardLimitSwitchActive();
    }

    @Override
    public void setSteeringVelocity(int unitsPer100ms) {
        steeringMotor.setVelocity(unitsPer100ms);
    }

    @Override
    public void stopMotors() {
        steeringMotor.setPercent(0);
        drivingMotor.setPercent(0);
    }

    @Override
    public void resetSteeringEncoder() {
        steeringMotor.setPosition(0);
    }

    @Override
    public void resetDrivingEncoder() {
        drivingMotor.setPosition(0);
    }

    @Override
    public int getSteeringEncoderPulses() {
        return steeringMotor.getEncoderTicks();
    }

    @Override
    public int getDriveEncoderPulses() {
        return drivingMotor.getEncoderTicks();
    }
}
