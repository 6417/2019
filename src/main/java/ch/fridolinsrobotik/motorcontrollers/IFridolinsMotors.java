/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.ctre.phoenix.motorcontrol.IMotorController;

/**
 * Interface to merge the MotorController classes given by the manufacturers.
 */
public interface IFridolinsMotors {
     
    public void setVelocity(double velocity);

    public void setPosition(double position);

    public void enableForwardLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable);

    public void enableReverseLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable);

    public boolean isForwardLimitSwitchActive();

    public boolean isReverseLimitSwitchActive();

    public void setIdleMode(FridolinsIdleModeType type);

    public void follow(IFridolinsMotors master);

    public void setDirection(boolean forward);

    public void followDirection(FridolinsDirectionType type);

    public void setSensorDirection(boolean forward);

    public void setSensorPosition(int position);

    public int getEncoderTicks();

    public void factoryDefault();

    public void configSelectedFeedbackSensor(FridolinsFeedbackDevice device, int pidIdx, int timeoutMs);

    public void configOpenLoopRamp(double rampTime, int timeoutMs);
}
