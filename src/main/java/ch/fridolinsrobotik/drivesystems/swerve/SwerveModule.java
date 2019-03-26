/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableImpl;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A class for representing a Swerve module.
 * 
 * A Swerve module consists out of two motors. One motor rotates the wheel
 * around its Z-axis to steer and the other around its Y-axis to drive.
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
 */
public abstract class SwerveModule implements Sendable, AutoCloseable {

    private final SendableImpl m_sendableImpl;

    protected int steeringPulsesPerRotation = 1;
    protected int drivingPulsesPerRotation = 1;
    protected Vector2d naturalRotateVector = new Vector2d(0, 0);
    protected double driveSpeedPercentage = 0;
    protected double driveInverted = 1.0;
    protected int steeringPosition = 0;

    private SwerveModule() {
        m_sendableImpl = new SendableImpl(true);
        setName("SwerveModule");
    }

    public SwerveModule(Vector2d mountingPoint, int steeringPulsesPerRotation, int drivingPulsesPerRotation) {
        this();
        verify(mountingPoint, steeringPulsesPerRotation, drivingPulsesPerRotation);
        this.steeringPulsesPerRotation = steeringPulsesPerRotation;
        this.drivingPulsesPerRotation = drivingPulsesPerRotation;
        calculateNaturalRotationVector(mountingPoint);
    }

    /**
     * Verifies that mounting point is nonnull, throwing a NullPointerException if
     * it is. The method also verifies that the mounting point is not the origin
     * where a swerve module has not a natural rotating vector.
     * 
     * Verifies thath pulses per rotation (PPR) are not null to prevent division by
     * 0 errors in {@link SwerveModule#convertEncoderPulsesToRadians(int, int)}.
     * 
     * @throws NullPointerException     if the mounting point is null
     * @throws IllegalArgumentException if the mounting point is the origin or if
     *                                  any PPR is 0
     */
    private void verify(Vector2d mountingPoint, int steeringPulsesPerRotation, int drivingPulsesPerRotation) {
        if (mountingPoint != null && mountingPoint.magnitude() != 0 && steeringPulsesPerRotation != 0
                && drivingPulsesPerRotation != 0) {
            return;
        }

        if (mountingPoint == null) {
            throw new NullPointerException("Mounting point is missing");
        }

        StringBuilder sb = new StringBuilder();
        if (mountingPoint.magnitude() == 0) {
            sb.append("Mounting point must not be the origin. ");
        }

        if (steeringPulsesPerRotation == 0) {
            sb.append("Steering PPR must not be 0. ");
        }

        if (drivingPulsesPerRotation == 0) {
            sb.append("Driving PPR must not be 0");
        }

        throw new IllegalArgumentException(sb.toString());
    }

    public void setInvertedDrive(boolean inverted) {
        if (inverted) {
            this.driveInverted = -1.0;
        } else {
            this.driveInverted = 1.0;
        }
    }

    protected void calculateNaturalRotationVector(Vector2d mountingPoint) {
        // the natural rotate vector stands 90' on the mounting point
        // while the magnitude must remain 1 to count as the identity element
        // for further multiplication.

        // rotate the mounting point vector by 90' clockwise
        naturalRotateVector.x = mountingPoint.y;
        naturalRotateVector.y = -mountingPoint.x;

        // normalize the magnitude to 1
        naturalRotateVector.x /= mountingPoint.magnitude();
        naturalRotateVector.y /= mountingPoint.magnitude();
    }

    /**
     * Drive calculation method for Serve platform.
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
    protected void calculateSwerveMovement(double ySpeed, double xSpeed, double zRotation) {
        double wheelAngle = getSteeringAngle();

        /* the wheel angle as a vector representation */
        Vector2d wheelVector = new Vector2d(Math.cos(wheelAngle), Math.sin(wheelAngle));
        Vector2d rotationVector = new Vector2d(zRotation * naturalRotateVector.x, zRotation * naturalRotateVector.y);
        Vector2d striveVector = new Vector2d(xSpeed, ySpeed);
        Vector2d targetVector = new Vector2d(rotationVector.x + striveVector.x, rotationVector.y + striveVector.y);
        
        /*
         * if the target's vector's magnitude is 0 the drive speed is 0 and therefore
         * nothing must be calcualted.
         */
        if (targetVector.magnitude() == 0.0) {
            setDriveSpeedPercentage(0);
            return;
        }

        /*
         * Angle between wheel vector and target vector. Only the target vector's
         * magnitude is needed, since the wheel vector's magnitude is always 1.
         */
        double angleToSteer = Math.acos(targetVector.dot(wheelVector) / targetVector.magnitude());
        double steeringDirection = Math.signum(wheelVector.x * targetVector.y - wheelVector.y * targetVector.x);
        
        double driveDirection;
        /* if steering angle is bigger than 90' the opposite side (-180') is faster */
        if (angleToSteer > Math.PI / 2) {
            angleToSteer -= Math.PI;
            driveDirection = -1;
        } else {
            driveDirection = 1;
        }

        angleToSteer *= steeringDirection;
        
        setSteeringPosition(getSteeringEncoderPulses()
                + convertRadiansToEncoderPulses(angleToSteer, getSteeringPulsesPerRotation()));
        setDriveSpeedPercentage(driveDirection * targetVector.magnitude() * driveInverted);
    }

    /**
     * Feeds the calculated swerve movement values into the motor controllers.
     */
    public abstract void executeSwerveMovement();

    /**
     * Is used to check the homing sensor from different motor controllers
     * 
     * @return True when the homing sensor is active.
     */
    public abstract boolean homingSensorActive();

    /**
     * Sets the steering motor to velocity control mode instead of position control
     * mode. Can be used to home the steering wheel independent of resistance
     * applied to the wheel.
     * 
     * @param unitsPer100ms velocity in units per 100ms.
     */
    public abstract void setSteeringVelocity(int unitsPer100ms);

    public abstract void stopMotors();

    public abstract void resetSteeringEncoder();

    public abstract void resetDrivingEncoder();

    /**
     * @param steeringPulsesPerRotation Pulses per rotation of the wheel for a full
     *                                  turn around the wheel's Z-axis
     */
    public void setSteeringPulsesPerRotation(int steeringPulsesPerRotation) {
        this.steeringPulsesPerRotation = steeringPulsesPerRotation;
    }

    /**
     * @param drivingPulsesPerRotation Pulses per rotation of the wheel for a full
     *                                 turn around the wheel's Y-axis
     */
    public void setDrivingPulsesPerRotation(int drivingPulsesPerRotation) {
        this.drivingPulsesPerRotation = drivingPulsesPerRotation;
    }

    /**
     * Sets the driving speed of the Swerve module. Can be used to adjust the speed
     * of it.
     * 
     * @param driveSpeedPercentage Motor speed in percentage [-1..1].
     */
    protected void setDriveSpeedPercentage(double driveSpeedPercentage) {
        this.driveSpeedPercentage = driveSpeedPercentage;
    }

    /**
     * Sets the steering position of the Swerve module. Can be used to adjust the
     * position of it.
     * 
     * @param steeringPosition Absolute encoder position of the steering wheel.
     */
    protected void setSteeringPosition(int steeringPosition) {
        this.steeringPosition = steeringPosition;
    }

    /**
     * @return Pulses per rotation of the wheel for a full turn around the wheel's
     *         Z-axis
     */
    public int getSteeringPulsesPerRotation() {
        return this.steeringPulsesPerRotation;
    }

    /**
     * @return Pulses per rotation of the wheel for a full turn around the wheel's
     *         Z-axis
     */
    public int getDrivingPulsesPerRotation() {
        return this.drivingPulsesPerRotation;
    }

    /**
     * @return The natural rotation vector of the wheel when rotating the robot
     *         around its Z-axis.
     */
    protected Vector2d getRotateVector() {
        return this.naturalRotateVector;
    }

    /**
     * Calculates the steering angle.
     * 
     * @return Steering angle in radians.
     */
    public double getSteeringAngle() {
        return convertEncoderPulsesToRadians(getSteeringEncoderPulses(), getSteeringPulsesPerRotation());
    }

    /**
     * Calculates the driving angle.
     * 
     * @return Drive angle in radians.
     */
    public double getDriveAngle() {
        return convertEncoderPulsesToRadians(getDriveEncoderPulses(), getDrivingPulsesPerRotation());
    }

    /**
     * Returns the calculated driving speed in percentage
     * 
     * @return Driving speed in percentage.
     */
    public double getDriveSpeedPercentage() {
        return this.driveSpeedPercentage;
    }

    /**
     * Returns the calculated steering position in encoder pulses. Can be used to
     * feed the PID with position control.
     * 
     * @return Steering position in absolute encoder pulses.
     */
    public int getSteeringPosition() {
        return this.steeringPosition;
    }

    /**
     * Get the relative position of the steering wheel in encoder pulses
     * 
     * @return relative position of the steering wheel in encoder pulses
     */
    public abstract int getSteeringEncoderPulses();

    /**
     * Get the relative position of the drive wheel in encoder pulses
     * 
     * @return relative position of the drive wheel in encoder pulses
     */
    public abstract int getDriveEncoderPulses();

    protected double convertEncoderPulsesToRadians(int pulses, int pulsesPerRotation) {
        return (2 * Math.PI) / pulsesPerRotation * pulses;
    }

    protected int convertRadiansToEncoderPulses(double radians, int pulsesPerRotation) {
        return (int) (pulsesPerRotation / (2 * Math.PI) * radians);
    }

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
        builder.setSmartDashboardType("SwerveModule");
        builder.setSafeState(this::stopMotors);
        builder.addDoubleProperty("Steering Angle", () -> Math.toDegrees(this.getSteeringAngle()), null);
        builder.addDoubleProperty("Steering Angle Goal",
                () -> Math.toDegrees(
                        this.convertEncoderPulsesToRadians(this.getSteeringPosition(), this.steeringPulsesPerRotation)),
                null);
        builder.addDoubleProperty("Drive Speed", this::getDriveSpeedPercentage, null);
        builder.addBooleanProperty("Homing Sensor", this::homingSensorActive, null);
    }

    /*************************** Sendable part ends *****************************/

}
