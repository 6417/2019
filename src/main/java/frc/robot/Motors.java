/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ch.fridolinsrobotik.motorcontrollers.FridolinsDirectionType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsFeedbackDevice;
import ch.fridolinsrobotik.motorcontrollers.FridolinsIdleModeType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;

/**
 * Add your docs here.
 */
public class Motors {

    //Create Motors
    public static IFridolinsMotors cargoGripperMaster;
    public static IFridolinsMotors cargoGripperFollower;

    public static IFridolinsMotors hatchGripperMotor;

    public static WPI_TalonSRX cartMotor;

    public static IFridolinsMotors liftMaster;
    public static IFridolinsMotors liftFollower;

    public static IFridolinsMotors swerveDriveFrontRight;
    public static IFridolinsMotors swerveDriveFrontLeft;
    public static IFridolinsMotors swerveDriveBackRight;
    public static IFridolinsMotors swerveDriveBackLeft;
    public static IFridolinsMotors swerveAngleFrontRight;
    public static IFridolinsMotors swerveAngleFrontLeft;
    public static IFridolinsMotors swerveAngleBackRight;
    public static IFridolinsMotors swerveAngleBackLeft;

    public static ArrayList<IFridolinsMotors> swerveDriveMotors = new ArrayList<IFridolinsMotors>();
    public static ArrayList<IFridolinsMotors> swerveAngleMotors = new ArrayList<IFridolinsMotors>();
    public static ArrayList<IFridolinsMotors> swerveMotors = new ArrayList<IFridolinsMotors>();

    public static void initialize() {

        if(RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            //Initialize Motors
            IFridolinsMotors cargoGripperMotorRight = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_RIGHT_ID);
            IFridolinsMotors cargoGripperMotorLeft = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_LEFT_ID);

            cargoGripperMaster = cargoGripperMotorLeft;
            cargoGripperFollower = cargoGripperMotorRight;

            //Factory Default
            cargoGripperMaster.factoryDefault();
            cargoGripperFollower.factoryDefault();

            //Set Master and Follower
            cargoGripperFollower.follow(cargoGripperMaster);

            cargoGripperMaster.setDirection(false);
            cargoGripperFollower.followDirection(FridolinsDirectionType.followMaster);

            //Set Mode and Limit Switches
            cargoGripperMaster.setIdleMode(FridolinsIdleModeType.kCoast); 
            cargoGripperFollower.setIdleMode(FridolinsIdleModeType.kCoast);

            cargoGripperMaster.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
        }

        if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
            FridolinsTalonSRX liftLeft = new FridolinsTalonSRX(RobotMap.LIFTING_UNIT_MOTOR_LEFT_ID);
            FridolinsTalonSRX liftRight = new FridolinsTalonSRX(RobotMap.LIFTING_UNIT_MOTOR_RIGHT_ID);
            liftMaster = liftLeft;
            liftFollower = liftRight;

            liftLeft.configFactoryDefault();
            liftLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            liftLeft.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, liftRight.getDeviceID());
            liftLeft.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, liftRight.getDeviceID());

            liftRight.configFactoryDefault();
            liftRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            liftRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

            liftFollower.follow(liftMaster);
            liftFollower.followDirection(FridolinsDirectionType.invertMaster);
            
        }

        if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            //Initialize Motors
            hatchGripperMotor = new FridolinsTalonSRX(RobotMap.HATCH_GRIPPER_MOTOR_ID);

            //Facory Default
            hatchGripperMotor.factoryDefault();

            //Set Mode and Limit Switches
            hatchGripperMotor.setIdleMode(FridolinsIdleModeType.kBrake);
            hatchGripperMotor.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
            hatchGripperMotor.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
        }

        if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {

            //Initialize Motors
            swerveDriveFrontRight = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_FRONT_RIGHT_ID);
            swerveDriveFrontLeft = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_FRONT_LEFT_ID);
            swerveDriveBackRight = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_BACK_RIGHT_ID);
            swerveDriveBackLeft = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_BACK_LEFT_ID);
            swerveAngleFrontRight = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_FRONT_RIGHT_ID);
            swerveAngleFrontLeft = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_FRONT_LEFT_ID);
            swerveAngleBackRight = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_BACK_RIGHT_ID);
            swerveAngleBackLeft = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_BACK_LEFT_ID);

            swerveDriveMotors.add(swerveDriveFrontRight);
            swerveDriveMotors.add(swerveDriveFrontLeft);
            swerveDriveMotors.add(swerveDriveBackRight);
            swerveDriveMotors.add(swerveDriveBackLeft);

            
            swerveAngleMotors.add(swerveAngleFrontRight);
            swerveAngleMotors.add(swerveAngleFrontLeft);
            swerveAngleMotors.add(swerveAngleBackRight);
            swerveAngleMotors.add(swerveAngleBackLeft);

            swerveMotors.addAll(swerveDriveMotors);
            swerveMotors.addAll(swerveAngleMotors);
            
            for (IFridolinsMotors motor : swerveMotors) {
                motor.factoryDefault();
                motor.setIdleMode(FridolinsIdleModeType.kBrake);
                motor.setDirection(true);
                motor.setSensorDirection(true);
                motor.configSelectedFeedbackSensor(FridolinsFeedbackDevice.QuadEncoder, 0, 0);
                motor.configOpenLoopRamp(0, 0);
                
            }

            for (IFridolinsMotors motor : swerveDriveMotors) {
                motor.setDirection(false);
            }

            for (IFridolinsMotors motor : swerveAngleMotors) {
                motor.setDirection(false);
                motor.setSensorDirection(false);
            }

        }
        
        if(RobotMap.CART_SUBYSTEM_IS_IN_USE) {
            WPI_TalonSRX cartMotor = new WPI_TalonSRX(RobotMap.CART_MOTOR_ID);

            cartMotor.configFactoryDefault();
            cartMotor.setNeutralMode(NeutralMode.Brake);
            cartMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
            cartMotor.setSensorPhase(false);
            cartMotor.setInverted(false);
            cartMotor.configClosedloopRamp(0);
            cartMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
            cartMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
            cartMotor.configNominalOutputForward(0, 30);
            cartMotor.configNominalOutputReverse(0, 30);    
            cartMotor.configPeakOutputForward(1, 30);
            cartMotor.configPeakOutputReverse(-1, 30);
            cartMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
            cartMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
            cartMotor.configClearPositionOnLimitR(true, 0);
            cartMotor.configClearPositionOnLimitF(true, RobotMap.CART_DRIVE_LENGTH);
            cartMotor.selectProfileSlot(0, 0);
		    cartMotor.config_kF(0, 0.02, 30);
		    cartMotor.config_kP(0, 35.0*1023/38000, 30);
		    cartMotor.config_kI(0, 0.004, 30);
            cartMotor.config_kD(0, 350.0*1023/38000, 30);
            cartMotor.config_IntegralZone(0, 500);
            cartMotor.configMotionCruiseVelocity(RobotMap.CART_ENCODER_UNITS_PER_100_MS / 20, 30);
            cartMotor.configMotionAcceleration(RobotMap.CART_ENCODER_UNITS_PER_100_MS / 20, 30);
            cartMotor.setSelectedSensorPosition(0, 0, 30);

            // cartMotor = new FridolinsTalonSRX(RobotMap.CART_MOTOR_ID);
        }
    }
}
