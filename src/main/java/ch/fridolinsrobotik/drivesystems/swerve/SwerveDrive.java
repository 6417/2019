/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.ArrayList;

import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableImpl;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A class for driving Swerve drive platforms.
 *
 * <p>
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * https://upload.wikimedia.org/wikipedia/commons/2/2f/RPY_angles_of_airplanes.png.
 * </p>
 * <p>
 * The positive X axis points ahead, the positive Y axis points right, and the
 * positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 * </p>
 * <p>
 * Inputs smaller then
 * {@value ch.fridolinsrobotik.drivesystems.swerve.SwerveDrive#kDefaultDeadband}
 * will be set to 0, and larger values will be scaled so that the full range is
 * still used. This deadband value can be changed with {@link #setDeadband}.
 * </p>
 */
public class SwerveDrive extends MotorSafety implements Sendable, AutoCloseable {
    public static final double kDefaultDeadband = 0.02;
    public static final double kDefaultSpeedFactor = 1.0;

    protected double m_deadband = kDefaultDeadband;
    protected double m_speedFactor = kDefaultSpeedFactor;

    private static int instances;

    private final SendableImpl m_sendableImpl;

    private ArrayList<SwerveModule> swerveModules = new ArrayList<>();

    public SwerveDrive() {
        m_sendableImpl = new SendableImpl(true);
        instances++;
        setSafetyEnabled(true);
        setName("SwerveDrive", instances);
    }

    public void addSwerveModule(SwerveModule module) {
        if (module == null) {
            DriverStation.reportError("[" + getName() + "] Swerve module passed is null!", true);
            return;
        }
        addChild(module);
        this.swerveModules.add(module);
    }

    public ArrayList<SwerveModule> getSwerveModules() {
        return this.swerveModules;
    }

    public double getDeadband() {
        return this.m_deadband;
    }

    public void setDeadband(double m_deadband) {
        this.m_deadband = m_deadband;
    }

    public double getSpeedFactor() {
        return this.m_speedFactor;
    }

    public void setSpeedFactor(double speedFactor) {
        this.m_speedFactor = speedFactor;
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>
     * Angles are measured clockwise from the positive X axis. The robot's speed is
     * independent from its angle or rotation rate.
     *
     * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
     *                  positive.
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
     *                  positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                  Clockwise is positive.
     */
    public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>
     * Angles are measured clockwise from the positive X axis. The robot's speed is
     * independent from its angle or rotation rate.
     *
     * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
     *                  positive.
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
     *                  positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                  Clockwise is positive.
     * @param gyroAngle The current angle reading from the gyro in degrees around
     *                  the Z axis. Use this to implement field-oriented controls.
     */
    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {

        ySpeed = Algorithms.limit(ySpeed, -1, 1);
        xSpeed = Algorithms.limit(xSpeed, -1, 1);
        zRotation = Algorithms.limit(zRotation, -1, 1);

        Vector2d striveVector = new Vector2d(xSpeed, ySpeed);
        striveVector.rotate(-gyroAngle);

        /*
         * apply deadband to rotation to prevent jittering of the steering.
         */
        if (Math.abs(zRotation) < m_deadband) {
            zRotation = 0;
        } else if (zRotation >= m_deadband) {
            zRotation = Algorithms.scale(zRotation, m_deadband, 1, 0, 1);
        } else {
            zRotation = Algorithms.scale(zRotation, -1, m_deadband, -1, 0);
        }

        /*
         * apply deadband without disturbing the steering angle to prevent abrupt
         * stopping.
         */
        if(striveVector.magnitude() >= m_deadband) {
            striveVector = Algorithms.scale(striveVector, m_deadband, 1, 0, 1);
        }

        for (SwerveModule module : this.swerveModules) {
            
            /*
             * Reduce speed to zero while keeping the last steering position.
             */
            if (striveVector.magnitude() < m_deadband && Math.abs(zRotation) < m_deadband) {
                module.setDriveSpeedPercentage(0);
            } else {
                module.calculateSwerveMovement(striveVector.y, striveVector.x, zRotation);
                module.setDriveSpeedPercentage(module.getDriveSpeedPercentage() * this.m_speedFactor);
            }
        }

        normalize();

        for (SwerveModule module : this.swerveModules) {
            module.executeSwerveMovement();
        }
        feed();
    }

    /**
     * Drive method for Swerve platform.
     *
     * <p>
     * Angles are measured counter-clockwise from straight ahead. The speed at which
     * the robot drives (translation) is independent from its angle or rotation
     * rate.
     *
     * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is
     *                  positive.
     * @param angle     The angle around the Z axis at which the robot drives in
     *                  degrees [-180..180].
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                  Clockwise is positive.
     */
    public void drivePolar(double magnitude, double angle, double zRotation) {
        driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)), magnitude * Math.cos(angle * (Math.PI / 180.0)),
                zRotation, 0.0);
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    protected void normalize() {
        double maxMagnitude = 0;
        for (int i = 0; i < this.swerveModules.size(); i++) {
            double temp = Math.abs(this.swerveModules.get(i).getDriveSpeedPercentage());
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < this.swerveModules.size(); i++) {
                SwerveModule module = this.swerveModules.get(i);
                module.setDriveSpeedPercentage(module.getDriveSpeedPercentage() / maxMagnitude);
            }
        }
    }

    /************************* MotorSafety part ends ***************************/

    public void stopMotor() {
        for (SwerveModule module : this.swerveModules) {
            module.stopMotors();
        }
        feed();
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }

    /************************** MotorSafety part ends ****************************/

    /*************************** Sendable part begin *****************************/

    @Override
    public void close() {
        m_sendableImpl.close();
    }

    @Override
    public final synchronized String getName() {
        return m_sendableImpl.getName();
    }

    @Override
    public final synchronized void setName(String name) {
        m_sendableImpl.setName(name);
    }

    /**
     * Sets the name of the sensor with a channel number.
     *
     * @param moduleType A string that defines the module name in the label for the
     *                   value
     * @param channel    The channel number the device is plugged into
     */
    protected final void setName(String moduleType, int channel) {
        m_sendableImpl.setName(moduleType, channel);
    }

    /**
     * Sets the name of the sensor with a module and channel number.
     *
     * @param moduleType   A string that defines the module name in the label for
     *                     the value
     * @param moduleNumber The number of the particular module type
     * @param channel      The channel number the device is plugged into (usually
     *                     PWM)
     */
    protected final void setName(String moduleType, int moduleNumber, int channel) {
        m_sendableImpl.setName(moduleType, moduleNumber, channel);
    }

    @Override
    public final synchronized String getSubsystem() {
        return m_sendableImpl.getSubsystem();
    }

    @Override
    public final synchronized void setSubsystem(String subsystem) {
        m_sendableImpl.setSubsystem(subsystem);
    }

    /**
     * Add a child component.
     *
     * @param child child component
     */
    protected final void addChild(Object child) {
        m_sendableImpl.addChild(child);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
    }

    /*************************** Sendable part ends *****************************/
}
