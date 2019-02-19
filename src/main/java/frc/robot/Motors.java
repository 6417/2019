/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ch.fridolinsrobotik.motorcontrollers.FridolinsDirectionType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsIdleModeType;
import ch.fridolinsrobotik.motorcontrollers.FridolinsLimitSwitchPolarity;
import ch.fridolinsrobotik.motorcontrollers.FridolinsTalonSRX;
import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;

/**
 * Add your docs here.
 */
public class Motors {

    // Create Motors
    public static FridolinsTalonSRX cargoGripperMaster;
    public static FridolinsTalonSRX cargoGripperFollower;

    public static FridolinsTalonSRX hatchGripperMotor;

    public static FridolinsTalonSRX cartMotor;

    public static WPI_TalonSRX liftMaster;
    public static WPI_TalonSRX liftFollower;

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

        if (RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            // Initialize Motors
            FridolinsTalonSRX cargoGripperMotorRight = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_RIGHT_ID);
            FridolinsTalonSRX cargoGripperMotorLeft = new FridolinsTalonSRX(RobotMap.CARGO_GRIPPER_MOTOR_LEFT_ID);

            cargoGripperMaster = cargoGripperMotorLeft;
            cargoGripperFollower = cargoGripperMotorRight;

            // Factory Default
            cargoGripperMaster.factoryDefault();
            cargoGripperFollower.factoryDefault();

            // Set Master and Follower
            cargoGripperFollower.follow((WPI_TalonSRX)cargoGripperMaster);

            cargoGripperMaster.setDirection(false);
            cargoGripperFollower.setInverted(InvertType.FollowMaster);

            // Set Mode and Limit Switches
            cargoGripperMaster.setIdleMode(FridolinsIdleModeType.kCoast);
            cargoGripperFollower.setIdleMode(FridolinsIdleModeType.kCoast);

            cargoGripperMaster.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyClosed, true);
        }

        if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
            liftMaster = new WPI_TalonSRX(RobotMap.LIFTING_UNIT_MOTOR_LEFT_ID);
            liftFollower = new WPI_TalonSRX(RobotMap.LIFTING_UNIT_MOTOR_RIGHT_ID);
           
            // FridolinsTalonSRX liftLeft = new FridolinsTalonSRX(RobotMap.LIFTING_UNIT_MOTOR_LEFT_ID);
            // FridolinsTalonSRX liftRight = new FridolinsTalonSRX(RobotMap.LIFTING_UNIT_MOTOR_RIGHT_ID);
            // liftMaster = liftLeft;
            // liftFollower = liftRight;

            liftMaster.setSafetyEnabled(false);
            liftMaster.setExpiration(3);

            liftMaster.configFactoryDefault();
            liftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            liftMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 30);
            liftMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 30);
            liftMaster.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

            liftFollower.configFactoryDefault();
            liftFollower.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
            liftFollower.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
            /* Configure the right Talon's selected sensor to a QuadEncoder*/
            liftFollower.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
            /* Configure the Remote Talon's selected sensor as a remote sensor for the left Talon */
            liftMaster.configRemoteFeedbackFilter(
            liftFollower.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, 30);

            liftMaster.setInverted(true);
            liftFollower.setInverted(InvertType.OpposeMaster);
            // liftFollower.follow(liftMaster);


        } 

        if (RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
            // Initialize Motors
            hatchGripperMotor = new FridolinsTalonSRX(RobotMap.HATCH_GRIPPER_MOTOR_ID);

            // Facory Default
            hatchGripperMotor.factoryDefault();

            //Set Mode and Limit Switches
            hatchGripperMotor.setDirection(true);
            hatchGripperMotor.setSensorDirection(true);
            hatchGripperMotor.setIdleMode(FridolinsIdleModeType.kBrake);
            hatchGripperMotor.setIdleMode(FridolinsIdleModeType.kBrake);
            hatchGripperMotor.enableForwardLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);
            hatchGripperMotor.enableReverseLimitSwitch(FridolinsLimitSwitchPolarity.kNormallyOpen, true);

            hatchGripperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
            hatchGripperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
            hatchGripperMotor.configNominalOutputForward(0, 30);
            hatchGripperMotor.configNominalOutputReverse(0, 30);    
            hatchGripperMotor.configPeakOutputForward(1, 30);
            hatchGripperMotor.configPeakOutputReverse(-1, 30);

            hatchGripperMotor.selectProfileSlot(0, 0);
		    hatchGripperMotor.config_kF(0, 0.03196875, 30);
		    hatchGripperMotor.config_kP(0, 1.6, 30);
            hatchGripperMotor.config_kI(0, 0.05, 30);
            hatchGripperMotor.config_kD(0, 32, 30);
            hatchGripperMotor.config_IntegralZone(0, 500);
            hatchGripperMotor.configAllowableClosedloopError(0, 30, 30);
            hatchGripperMotor.configMotionCruiseVelocity(RobotMap.HATCH_GRIPPER_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS, 30);
            hatchGripperMotor.configMotionAcceleration(RobotMap.HATCH_GRIPPER_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS, 30);
        }

        if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {

            //Initialize Motors
            FridolinsTalonSRX talonSwerveDriveFrontRight = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_FRONT_RIGHT_ID);
            FridolinsTalonSRX talonSwerveDriveFrontLeft = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_FRONT_LEFT_ID);
            FridolinsTalonSRX talonSwerveDriveBackRight = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_BACK_RIGHT_ID);
            FridolinsTalonSRX talonSwerveDriveBackLeft = new FridolinsTalonSRX(RobotMap.SWERVE_DRIVE_BACK_LEFT_ID);
            FridolinsTalonSRX talonSwerveAngleFrontRight = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_FRONT_RIGHT_ID);
            FridolinsTalonSRX talonSwerveAngleFrontLeft = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_FRONT_LEFT_ID);
            FridolinsTalonSRX talonSwerveAngleBackRight = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_BACK_RIGHT_ID);
            FridolinsTalonSRX talonSwerveAngleBackLeft = new FridolinsTalonSRX(RobotMap.SWERVE_ANGLE_BACK_LEFT_ID);

            ArrayList<FridolinsTalonSRX> talonSwerveDriveMotors = new ArrayList<FridolinsTalonSRX>();
            ArrayList<FridolinsTalonSRX> talonSwerveAngleMotors = new ArrayList<FridolinsTalonSRX>();
            ArrayList<FridolinsTalonSRX> talonSwerveMotors = new ArrayList<FridolinsTalonSRX>();

            talonSwerveDriveMotors.add(talonSwerveDriveFrontLeft);
            talonSwerveDriveMotors.add(talonSwerveDriveFrontRight);
            talonSwerveDriveMotors.add(talonSwerveDriveBackRight);
            talonSwerveDriveMotors.add(talonSwerveDriveBackLeft);
            
            talonSwerveAngleMotors.add(talonSwerveAngleFrontLeft);
            talonSwerveAngleMotors.add(talonSwerveAngleFrontRight);
            talonSwerveAngleMotors.add(talonSwerveAngleBackRight);
            talonSwerveAngleMotors.add(talonSwerveAngleBackLeft);

            talonSwerveMotors.addAll(talonSwerveDriveMotors);
            talonSwerveMotors.addAll(talonSwerveAngleMotors);

            swerveDriveMotors.addAll(talonSwerveDriveMotors);
            swerveAngleMotors.addAll(talonSwerveAngleMotors);
            swerveMotors.addAll(talonSwerveMotors);

            for (FridolinsTalonSRX motor : talonSwerveMotors) {
                motor.configFactoryDefault(20);
			    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);

			    /* Set the peak and nominal outputs */
			    motor.configNominalOutputForward(0, 20);
			    motor.configNominalOutputReverse(0, 20);
			    motor.configPeakOutputForward(1, 20);
                motor.configPeakOutputReverse(-1, 20);
      
                /* Zero the sensor */
			    motor.setSelectedSensorPosition(0, 0, 20);
            }

            for (FridolinsTalonSRX motor : talonSwerveAngleMotors) {
                
                motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
                // motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 30);
                // motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);
                // motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 30);
                // motor.setStatusFramePeriod(StatusFrame.Status_1_General, 30);
                // motor.setControlFramePeriod(ControlFrame.Control_3_General, 30);
    
                /* PID settings */
                motor.selectProfileSlot(0, 0);
                motor.config_IntegralZone(0, 100);
                motor.config_kF(0, 0);
                motor.config_kP(0, 0.6);
                motor.config_kI(0, 0.16);
                motor.config_kD(0,4);
                motor.configClosedLoopPeakOutput(0, 0.5);

                motor.setSensorPhase(false);
                motor.setInverted(false);
                motor.set(ControlMode.Position, 0);
            }

            talonSwerveDriveFrontRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 30);
            talonSwerveDriveFrontRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 30);

            talonSwerveDriveBackLeft.setDirection(false);
            talonSwerveDriveBackRight.setDirection(true);
            talonSwerveDriveFrontLeft.setDirection(false);
            talonSwerveDriveFrontRight.setDirection(false);

            swerveDriveFrontRight = talonSwerveDriveFrontRight;
            swerveDriveFrontLeft = talonSwerveDriveFrontLeft;
            swerveDriveBackRight = talonSwerveDriveBackRight;
            swerveDriveBackLeft = talonSwerveDriveBackLeft;
            swerveAngleFrontRight = talonSwerveAngleFrontRight;
            swerveAngleFrontLeft = talonSwerveAngleFrontLeft;
            swerveAngleBackRight = talonSwerveAngleBackRight;
            swerveAngleBackLeft = talonSwerveAngleBackLeft;

        }
        
        if(RobotMap.CART_SUBYSTEM_IS_IN_USE) {
            FridolinsTalonSRX talonCart = new FridolinsTalonSRX(RobotMap.CART_MOTOR_ID);

            talonCart.configFactoryDefault();
            talonCart.setNeutralMode(NeutralMode.Brake);
            talonCart.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
            talonCart.setSensorPhase(false);
            talonCart.setInverted(true);
            talonCart.configOpenloopRamp(1);
            talonCart.configClosedloopRamp(0);
            talonCart.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
            talonCart.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
            talonCart.configNominalOutputForward(0, 30);
            talonCart.configNominalOutputReverse(0, 30);    
            talonCart.configPeakOutputForward(1, 30);
            talonCart.configPeakOutputReverse(-1, 30);
            talonCart.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, RobotMap.CART_REMOTE_LIMIT_SWITCH_ID);
            talonCart.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, RobotMap.CART_REMOTE_LIMIT_SWITCH_ID);
            talonCart.selectProfileSlot(0, 0);
		    talonCart.config_kF(0, 1 * 1023.0 / RobotMap.CART_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS, 30);
		    talonCart.config_kP(0, 0.8, 30);
		    talonCart.config_kI(0, 0, 30);
            talonCart.config_kD(0, 80, 30);
            talonCart.config_IntegralZone(0, 500);
            talonCart.configMotionCruiseVelocity(RobotMap.CART_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS / 2, 30);
            talonCart.configMotionAcceleration(RobotMap.CART_MAX_VELOCITY_ENCODER_UNITS_PER_100_MS, 30);
            talonCart.setSelectedSensorPosition(0, 0, 30);

            cartMotor = talonCart;

        } else if(RobotMap.CART_TESTSUBYSTEM_IS_IN_USE) {
            FridolinsTalonSRX talonTestCart = new FridolinsTalonSRX(RobotMap.CART_MOTOR_ID);
            talonTestCart.configFactoryDefault();
            talonTestCart.setNeutralMode(NeutralMode.Coast);
            talonTestCart.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
            talonTestCart.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            talonTestCart.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            talonTestCart.configClearPositionOnLimitR(false, 30);
            talonTestCart.configClearPositionOnLimitF(false, 30);
            talonTestCart.setInverted(false);
            talonTestCart.setSensorPhase(true);

            cartMotor = talonTestCart;
        }
    }
}
