/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import javax.naming.NotContextException;

import com.revrobotics.CANSparkMax;
/**
 * Add your docs here.
 */
public class FridolinsSparkMAX extends CANSparkMax implements IFridolinsMotors {

    public FridolinsSparkMAX(int deviceID, com.revrobotics.CANSparkMaxLowLevel.MotorType type) {

        super(deviceID, type);

    }

    @Override
    public void setVelocity(double velocity) {

        this.set(velocity);

    }

    @Override
    public void setPosition(double position) {

    }

    private LimitSwitchPolarity convertFridolinLimitSwitchPolarityToSparkMaxPolarity(FridolinsLimitSwitchPolarity polarity) {
        switch(polarity) {
            case kNormallyOpen:
                return LimitSwitchPolarity.kNormallyOpen;
            case kNormallyClosed:
                return LimitSwitchPolarity.kNormallyClosed;
            default:
                return LimitSwitchPolarity.kNormallyOpen;
        }
    }

    @Override
    public void enableForwardLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable) {
        super.getForwardLimitSwitch(
            convertFridolinLimitSwitchPolarityToSparkMaxPolarity(polarity)
        ).enableLimitSwitch(enable);
    }

    @Override
    public void enableReverseLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable) {
        super.getReverseLimitSwitch(
            convertFridolinLimitSwitchPolarityToSparkMaxPolarity(polarity)
        ).enableLimitSwitch(enable);
    }

    private IdleMode convertFridolinIdleModeType(FridolinsIdleModeType type) {
		switch(type) {
            case kBrake:
                return IdleMode.kBrake;
			default:
				return IdleMode.kCoast;
		}
	}


	@Override
	public void setIdleMode(FridolinsIdleModeType type) {
		super.setIdleMode(convertFridolinIdleModeType(type));
	}

    @Override
    public double getEncoderTicks() {
        super.getEncoder().getPosition();
        return getEncoder().getPosition();
    }

    @Override
    public boolean isForwardLimitSwitchActive() {
        throw new Error("NotImplemented");
    }

    @Override
    public boolean isReverseLimitSwitchActive() {
        throw new Error("NotImplemented");
    }

    @Override
    public void follow(IFridolinsMotors master) {
        throw new Error("NotImplemented");
    }

    @Override
    public void setDirection(boolean forward) {
        throw new Error("NotImplemented");
    }

    @Override
    public void followDirection(FridolinsDirectionType type) {
        throw new Error("NotImplemented");
    }

    @Override
    public void factoryDefault() {
        throw new Error("NotImplemented");
    }


}
