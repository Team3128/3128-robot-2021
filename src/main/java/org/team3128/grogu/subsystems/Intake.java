package org.team3128.grogu.subsystems;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.AsynchronousPid;
import org.team3128.common.control.motion.RamseteController;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.Trajectory.State;
import org.team3128.common.drive.AutoDriveSignal;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.NarwhalUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.utility.Log;
import org.team3128.common.drive.Drive;

public class Intake implements Subsystem {

    private enum IntakeState {
        TOP, BOTTOM;
    }

    public static final Intake instance = new Intake();
    private LazyTalonSRX ARM_MOTOR;
    private LazyVictorSPX BRUSH_MOTOR, INTAKE_MOTOR;

    private DigitalInput LIMIT_SWITCH_TOP, LIMIT_SWITCH_BOTTOM;

    private IntakeState intakeState; 

    public static Intake getInstance() { 
        return instance;
    }

    public Intake() {
        configMotors();
        configSensors();

        intakeState = IntakeState.BOTTOM;
    }

    private void configMotors() {
        ARM_MOTOR = new LazyTalonSRX(Constants.IntakeConstants.ARM_MOTOR_ID);
        BRUSH_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_ID);
        INTAKE_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);

        ARM_MOTOR.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);

    }
    
    private void configSensors() {
       
    }

    @Override
    public void periodic() {
        if (isBottomTriggered() && intakeState == IntakeState.TOP) {
            intakeState = IntakeState.BOTTOM;
            stopArm();
        } else if (isTopTriggered() && intakeState == IntakeState.BOTTOM) {
            intakeState = IntakeState.TOP;
            stopArm();
        }
    }

    public void runIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void runIntakeOpp() {
        //INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void stopIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, 0);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmDown() {
        if (intakeState == IntakeState.BOTTOM)
            ARM_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmUp() {
        if (intakeState == IntakeState.TOP)
            ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmUpAuto() {
        ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER_AUTO);
    }

    public void stopArm() {
        ARM_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public boolean isTopTriggered() {
        return LIMIT_SWITCH_TOP.get();
    }

    public boolean isBottomTriggered() {
        return LIMIT_SWITCH_BOTTOM.get();
    }
}
