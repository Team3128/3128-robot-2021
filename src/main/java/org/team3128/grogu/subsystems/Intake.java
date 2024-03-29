package org.team3128.grogu.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {

    private enum IntakeState {
        TOP, BOTTOM;
    }

    public static final Intake instance = new Intake();
    private LazyTalonSRX ARM_MOTOR;
    private LazyVictorSPX BRUSH_MOTOR_1, BRUSH_MOTOR_2, INTAKE_MOTOR;

    private DigitalInput LIMIT_SWITCH_TOP, LIMIT_SWITCH_BOTTOM;

    public IntakeState intakeState; 

    public static Intake getInstance() { 
        return instance;
    }

    public Intake() {
        configMotors();
        configSensors();

        intakeState = IntakeState.TOP;
    }

    private void configMotors() {
        ARM_MOTOR = new LazyTalonSRX(Constants.IntakeConstants.ARM_MOTOR_ID);
        BRUSH_MOTOR_1 = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_1_ID);
        BRUSH_MOTOR_2 = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_2_ID);
        INTAKE_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);

        ARM_MOTOR.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);
        BRUSH_MOTOR_2.set(ControlMode.Follower, Constants.IntakeConstants.BRUSH_MOTOR_1_ID);

    }
    
    private void configSensors() {
        //TODO: Set constants to real DigitalInput channels
        LIMIT_SWITCH_TOP = new DigitalInput(Constants.IntakeConstants.TOP_LIMIT_SWITCH_ID);
        LIMIT_SWITCH_BOTTOM = new DigitalInput(Constants.IntakeConstants.BOTTOM_LIMIT_SWITCH_ID);
    }

    @Override
    public void periodic() {
        if (isBottomTriggered() && intakeState == IntakeState.TOP) {
            intakeState = IntakeState.BOTTOM;
            //stopArm();
        } else if (isTopTriggered() && intakeState == IntakeState.BOTTOM) {
            intakeState = IntakeState.TOP;
            //stopArm();
        }
    }

    public void runIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR_1.set(ControlMode.PercentOutput, -Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void runIntakeOpp() {
        //INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR_1.set(ControlMode.PercentOutput, Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void stopIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, 0);
        BRUSH_MOTOR_1.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmUp() {
        if (intakeState == IntakeState.BOTTOM)
            ARM_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmDown() {
        if (intakeState == IntakeState.TOP)
            ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArm() {
        if (intakeState == IntakeState.BOTTOM)
            ARM_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
        else {
            ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
        }
    }

    public void moveArmUpAuto() {
        ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER_AUTO);
    }

    public void stopArm() {
        ARM_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public boolean isTopTriggered() {
        return !LIMIT_SWITCH_TOP.get();
    }

    public boolean isBottomTriggered() {
        return !LIMIT_SWITCH_BOTTOM.get();
    }
}
