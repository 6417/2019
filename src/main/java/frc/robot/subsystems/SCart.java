/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;

import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.sensors.utils.EncoderConverter;
import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Susystem to move the tower across the robot
 */
public class SCart extends Subsystem {

  FridolinsTalonSRX motor;
  EncoderConverter encoderConverter;

  /**
   * maximum travel of the cart until it hits the forwardLimitSwitch in cm
   * (Centimeter)
   */
  private double maximumTravel;

  /**
   * Distance before end or beginning of the cart slider until it scales down the
   * speed. E.g., speedEdgeWindow = 10 =>
   * 0------10cm-------maximumTravel-10cm------maximumTravel. î--------î  
   * -------------------î window window
   */
  private double speedEdgeWindow;

  /**
   * Distance before reaching the position and slowing down the speed
   */
  private double speedPositionSlowDownWindow;
  /**
   * matching window in cm (Centimeters) in which the encoder must lie to have
   * reached its target
   */
  private double positioningWindow;

  public SCart() {
    super();
    FridolinsTalonSRX motor = new FridolinsTalonSRX(RobotMap.CART_MOTOR_ID);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configClearPositionOnLimitR(true, 0);
    // TODO check if sensor is running in correct direction
    motor.configSelectedFeedbackCoefficient(1);
    motor.setNeutralMode(NeutralMode.Brake);

    this.motor = motor;

    this.encoderConverter = new EncoderConverter();
    this.encoderConverter.setDistancePerPulse(RobotMap.CART_ENCODER_DISTANCE_PER_PULSE);

    maximumTravel = 100000;
    speedEdgeWindow = 1000;
    speedPositionSlowDownWindow = 1000;
    positioningWindow = 100;
  }

  private double getDistanceTraveled() {
    return this.encoderConverter.getDistance(motor.getEncoderTicks());
  }

  /**
   * Scales the speed for the cart according to its position on the robot. The
   * closer the cart is at the edge, the slower it runs.
   * 
   * @param speed
   * @return scaled speed
   */
  private double scaleEdgeSpeed(double speed) {
    double distanceTraveled = getDistanceTraveled();
    double distanceToLimit;

    /**
     * check if distance traveld between beginning and speedEdgeWindow or greater
     * than maximumTravel-speedEdgeWindow
     */
    if (distanceTraveled <= speedEdgeWindow) {
      distanceToLimit = distanceTraveled;
    } else if (distanceTraveled >= maximumTravel - speedEdgeWindow) {
      distanceToLimit = maximumTravel - distanceTraveled;
    } else {
      return speed;
    }
    
    double x = Algorithms.scale(distanceToLimit, 0, speedEdgeWindow, 0, 1);
    speed *= Algorithms.easeInOut(x, 1.5);
    speed = Math.max(0.1, speed);
    return speed;
  }

  /**
   * Scales the speed for the cart according to its position on the robot and the target position.
   * 
   * @param speed desired speed
   * @param position target position
   * @return scaled speed
   */
  private double scalePositioningSpeed(double speed, double position) {
    double distanceTraveled = getDistanceTraveled();
    double distanceToPosition = Math.abs(distanceTraveled - position);

    /**
     * check if distance traveld between beginning and speedEdgeWindow or greater
     * than maximumTravel-speedEdgeWindow
     */
    if (distanceToPosition <= speedPositionSlowDownWindow) {
      double x = Algorithms.scale(distanceToPosition, 0, speedPositionSlowDownWindow, 0, 1);
      speed *= Algorithms.easeInOut(x,  1.5);
      speed = Math.max(0.1, speed);
    }
    
    return speed;
  }

  /**
   * Moves the motor backwards towards its zero point
   */
  public void moveToZeroPoint() {
    motor.setVelocity(-0.1);
  }

  /**
   * Move the cart to a desired position in cm measured from the zero point
   * 
   * @param position in cm (Centimeter)
   */
  public void moveToPosition(double position) {
    double velocity = 0;
    double distanceTraveled = getDistanceTraveled();

    if (distanceTraveled < position) {
      velocity = 1;
    } else if (distanceTraveled > position) {
      velocity = -1;
    }

    /**
     * Take the lower of both scalings. Overlaying scalings would result in too low speed.
     */
    velocity = Math.min(scaleEdgeSpeed(velocity), scalePositioningSpeed(velocity, position));
    motor.setVelocity(velocity);
    System.out.println("Motor Speed after scaling:" + velocity);
  }

   /**
   * Checks if the limit switches are pressed of the cart.
   */
  public boolean isLimitSwitchReached() {
    if (motor.getSensorCollection().isFwdLimitSwitchClosed() || motor.getSensorCollection().isRevLimitSwitchClosed()) {
      return true;
    }
    return false;
  }

  /**
   * determines if the cart has reached its desired position (in cm) or hit the limit switches
   */
  public boolean isPositionReached(double position) {
    double distanceTraveled = getDistanceTraveled();
    boolean limitSwitchHit = false;
    if(position <= 0 && motor.getSensorCollection().isRevLimitSwitchClosed()) {
      limitSwitchHit = true;
    } else if (position >= maximumTravel && motor.getSensorCollection().isFwdLimitSwitchClosed()) {
      limitSwitchHit = true;
    }
    if (getDistanceTraveled() > position - positioningWindow && getDistanceTraveled() < position + positioningWindow || limitSwitchHit) {
      return true;
    }
    return false;
  }

  public void stopMotor() {
    motor.setVelocity(0);
  }

  public void zeroPositionReached() {
    motor.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
  }
}
