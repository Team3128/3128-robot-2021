package org.team3128.grogu.subsystems;

import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.test_suite.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.team3128.common.utility.datatypes.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.RobotConstants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

// the superclass is purely for semantic purposes
public class Constants extends RobotConstants {

        public static class RamseteConstants {
                public static final double ksVolts = 0.8;//0.1 0.5
                public static final double kvVoltSecondsPerMeter = 0.5;//0.5 0.5
                public static final double kaVoltSecondsSquaredPerMeter = 0.05;//0.1  0.05
                public static final double kRamseteB = 5; //5  1.5
                public static final double kRamseteZeta = 0.7; //0.7  0.1
                public static final double kPDriveVel = 0.5;

                public static final double maxVelocity = 1.75; //1.5
                public static final double maxAcceleration = 1.5; //1.5
                
                public static final double kTrackwidthMeters = 0.66;
                public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);
        }


        public static class GameConstants {
                public static final double SHOOTER_TARGET_HEIGHT = 98.25; // height of target in INCHES
                public static final double BALL_HEIGHT = 9; // height of the ball in centimeters
        }

        public static class MechanismConstants {
                public static final double ENCODER_RESOLUTION_PER_ROTATION = 2048;
                public static final double inchesToMeters = 0.0254;
                public static final double DT = 0.005; // time between update() method calls for mechanisms
        }

        public static class DriveConstants {
                // conversion from inches per s to native units per 100 ms
                public static final double kDriveInchesPerSecPerNUp100ms = (1000d / 1)
                                * (1 / MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * (Constants.DriveConstants.WHEEL_DIAMETER * Math.PI)
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION / 100; // a fairly basic relationship between tangential and rotational speed: NU/100ms * 1000ms/1second * 1/(encoder resolution) * CIRCUM * (relation between encoder rotations and wheel rotations) = in/s
                
                // conversion from native units to inches
                public static final double kDriveNuToInches = (1
                                / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * Constants.DriveConstants.WHEEL_DIAMETER * Math.PI
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION;

                public static final NeutralMode DRIVE_IDLE_MODE = NeutralMode.Brake;

                public static final double ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION = 64 / 8; // basically your gearing. Ask Mech for rpm ratio: 8

                public static final double WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION = 1
                                / Constants.DriveConstants.ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION;

                public static final int RIGHT_DRIVE_FRONT_ID = 2;
                public static final int RIGHT_DRIVE_MIDDLE_ID = 3;

                public static final int LEFT_DRIVE_FRONT_ID = 0;
                public static final int LEFT_DRIVE_MIDDLE_ID = 1;

                public static final int DRIVE_HIGH_SPEED = 140; // Empirical Max Linear Speed: TBD in/s

                public static final double WHEEL_DIAMETER = 4; // 3.55 effective wheel diameter (measure first then tune this number until distances are accurate)

                public static final double LEFT_SPEEDSCALAR = 1.0; // purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)
                public static final double RIGHT_SPEEDSCALAR = 1.0;// purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)

                public static final double DRIVE_ACCEL_LIMIT = 120; // Ballpark estimates from mech (Be conservative unless you really need the quick auto paths)
                public static final double DRIVE_JERK_LIMIT = 2000; // Ballpark estimates (Be conservative)

                public static double K_AUTO_RIGHT_P = 0.00007; // 0.00065
                public static double K_AUTO_RIGHT_D = 0.000;
                public static double K_AUTO_RIGHT_F = 1 / 145.9150145782 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static double K_AUTO_LEFT_P = 0.00007;
                public static double K_AUTO_LEFT_D = 0.000; // 0.0001
                public static double K_AUTO_LEFT_F = 1 / 140.8705712261 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static final double K_HOLD_P = 4;

                public static final double kS = 0.178;
                public static final double kV = 0.055;//0.0516;
                public static final double kA = 0.00679;
                public static final double kP = 0.0013;

        }

        public static class AutonomousDriveConstants {
                public static final double TRACK_RADIUS = 23.89231390411386;
                public static final double MIN_TURNING_RADIUS = 40;
                public static final double MIN_PATH_SPEED = 20;
                public static final double MAX_PATH_SPEED = 120;
                public static final double MIN_LOOKAHEAD_DISTANCE = 14;
                public static final double MAX_LOOKAHEAD_DISTANCE = 30;
                public static final double MAX_TURN_ERROR = 2;
                public static final double MAX_PID_STOP_SPEED = 8;
        }

        public static class VisionConstants {
                public static final double BOTTOM_LIMELIGHT_HEIGHT = 28.85 * Length.in;
                public static final double PIVOT_HEIGHT = 16.0;
                public static final double BOTTOM_LIMELIGHT_ANGLE = 30.0 * Angle.DEGREES;
                public static final double BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT = 1 * Length.in;
                public static final int SAMPLE_RATE = 3;
                public static final double TX_THRESHOLD = 2; // the maximum error in tx where the shooter will be allowed to shoot
                public static final double TX_THRESHOLD_MAX = 5;
                public static final double TIME_TO_MAX_THRESHOLD = 5;

                public static final double TX_OFFSET = 0; // to offset alignment in either direction
                public static final PIDConstants VISION_PID = new PIDConstants(0, 0.01, 0.02, 0.00006);
                public static final PIDConstants BALL_PID = new PIDConstants(0.57, 0.02, 0.0, 0.00003);
                public static final PIDConstants BLIND_BALL_PID = new PIDConstants(0.23, 0, 0, 0);
                public static final double AUTO_ALIGN_TIMEOUT = 10; // seconds to wait before canceling 
        }


        public static class TestSuiteConstants {

                public static CanDevices intake;
                public static CanDevices rightDriveLeader;
                public static CanDevices rightDriveFollower;
                public static CanDevices feeder;
                public static CanDevices PDP;
                public static CanDevices leftDriveLeader;
                public static CanDevices leftDriveFollower;
                public static CanDevices armLeader;
                public static CanDevices armFollower;
                public static CanDevices shooterLeft;
                public static CanDevices gatekeeper;
                public static CanDevices shooterRight;
                public static CanDevices corner;
                
        }

        public static class ShooterConstants {
                public static final int SHOOTER_MOTOR_LEFT_ID = 8;
                public static final int SHOOTER_MOTOR_RIGHT_ID = 13;
                public static final int SHOOTER_SIDEKICK_ID = 42;
                
                // previous
                // public static final PIDConstants SHOOTER_PID = new PIDConstants(0, 16e-4, 5e-4, 4.20e-5);//0,0,0,4.20e-5//0, 2e-4, 0, 6e-6 //0.00040, 0, 5e-4//0, 0.0000457, 0, 2.25e-4
                
                // new
                // public static final PIDConstants SHOOTER_PID = new PIDConstants(0, 0.0457/5, 0, 4.20e-5);

                // new 2
                public static final PIDConstants SHOOTER_PID = new PIDConstants(0, 16e-6, 5e-6, 4.20e-6);
                
                public static final PIDConstants SIDEKICK_PID = new PIDConstants(0, 1e-6, 0, 0);

                public static final double SHOOTER_SATURATION_LIMIT = 5; // set limit on integral accumulation (in this case, 1 volt)
                // public static final double RPM_THRESHOLD = 200; // the maximum difference between an RPM and  the setpoint for a data point to be considered as a plataeu
                public static final double RPM_THRESHOLD_PERCENT = 0.05;
                // public static final double RPM_PLATEAU_THRESHOLD = 100; // the maximum difference between an RPM and  the setpoint for a data point to be considered as a plataeu
                public static final double RPM_PLATEAU_PERCENT = 0.02;
                public static final int PLATEAU_COUNT = 25; // 50 * 0.005s = 0.25 seconds of plateau required
                public static final double RPM_THRESHOLD_PERCENT_MAX = 0.1;
                public static final double TIME_TO_MAX_THRESHOLD = 5;

                public static final int CAN_TIMEOUT = 10;
        }

        public static class HopperConstants {
                public static final int HOPPER_MOTOR_1_ID = 6;
                public static final int HOPPER_MOTOR_2_ID = 19;
                
                public static final int BOTTOM_SENSOR_ID = 9;
                public static final int TOP_SENSOR_ID = 8;

                public static final double HOPPER_MOTOR_POWER = -0.4;
                public static final double HOPPER_MOTOR_2_POWER = 0.08;
        }

        public static class IntakeConstants {
                public static final int ARM_MOTOR_ID = 7;
                public static final int BRUSH_MOTOR_ID = 5;
                public static final int INTAKE_MOTOR_ID = 4;

                public static final int TOP_LIMIT_SWITCH_ID = 0;
                public static final int BOTTOM_LIMIT_SWITCH_ID = 1;

                public static final NeutralMode ARM_NEUTRAL_MODE = NeutralMode.Brake;

                public static final double INTAKE_MOTOR_POWER = -0.6;
                public static final double BRUSH_MOTOR_POWER = 0.3;
                public static final double ARM_MOTOR_POWER = 0.2;

                public static final double ARM_MOTOR_POWER_AUTO = 0.07;
        }

}