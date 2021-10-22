package org.team3128.grogu.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Hopper implements Subsystem {

    public enum HopperState {
        IDLE,INTAKING,SHOOTING;
    }

    public static final Hopper instance = new Hopper();
    private LazyTalonSRX HOPPER_MOTOR_1;
    private LazyCANSparkMax HOPPER_MOTOR_2;

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;

    private HopperState actionState = HopperState.IDLE;
    
    public int ballCount;

    private boolean wasTriggeredTop = false;
    private boolean wasTriggeredBottom = false;

    public boolean unshoot = false;
    public boolean intakeShooting = false;

    public boolean isShooterReady = false;

    public static Hopper getInstance() { 
        return instance;
    }

    public Hopper() {
        configMotors();
        configSensors();
        
        ballCount = 0;
    }

    private void configMotors() {
        HOPPER_MOTOR_1 = new LazyTalonSRX(Constants.HopperConstants.HOPPER_MOTOR_1_ID);
        HOPPER_MOTOR_2 = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_MOTOR_2_ID, MotorType.kBrushless);
    }
    
    private void configSensors() {
       BOTTOM_SENSOR = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID); 
       TOP_SENSOR = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
    }

    @Override
    public void periodic() {
        // Log.info("Hopper","ball count: " + ballCount);
        // Log.info("Hopper","action state: " + actionState);
        switch(actionState) {
            case IDLE:
                //stopIntake();
                //stopArm();
                stopHopper();
                if (getBottom() && ballCount < 3) {
                    setState(HopperState.INTAKING);
                    Log.info("Hopper","Ball is close enough to eat");
                }
                break;
            case INTAKING:
                intake();
                break;
            case SHOOTING:
                shoot();
                // if (getBottom())
                //     intakeShooting = true;
                // if(intakeShooting)
                //     intakeShoot();
                if (!getBottom() && wasTriggeredBottom) {
                    ballCount++;
                    Log.info("Hopper", "Sucked up another ball hwilst also shooting");
                }
                break;
        }
        wasTriggeredBottom = getBottom();
        wasTriggeredTop = getTop();
        //Log.info("hopper", "Ball in bottom = " + getBottom());
    }

    private void intake() {
        //Log.info("hopper","intaking");
        if (ballCount >= 2 || getTop()) {
            if (ballCount > 2)
                Log.info("Hopper","oopsie, should not be greater than 3");
            Log.info("Hopper Stomach","FULL!!!!");
            setState(HopperState.IDLE);
       } else {
            runHopper(1);
            if (!getBottom() && wasTriggeredBottom) {
                setState(HopperState.IDLE);
                ballCount++;
                Log.info("Hopper Stomach", "Has eaten 1 ball. Ball count = " + ballCount);
            }
       }

    }

    private void intakeShoot() {
        Log.info("hopper","intaking");
        if (ballCount >= 3 || getTop()) {
            if (ballCount > 3)
                Log.info("Hopper","oopsie, should not be greater than 3");
            Log.info("Hopper Stomach","FULL!!!!");
            intakeShooting = false;
       } else {
            runHopper(1);
            if (!getBottom() && wasTriggeredBottom) {
                intakeShooting = false;
                ballCount++;
                Log.info("Hopper Stomach", "Has eaten 1 ball. Ball count = " + ballCount);
            }
       }

    }

    private void shoot() {
        runHopper(1);
        //Log.info("hopper", "Shooting");
        if (wasTriggeredTop && !getTop()) {
            setState(HopperState.IDLE);
            ballCount--;
            Log.info("Hopper Stomach","Ejected one ball at a high velocity");
        }
        if (unshoot) {
            setState(HopperState.IDLE);
            unshoot = false;
            Log.info("Hopper Stomach", "Took some tums and I feel better");
        }
    }

    private boolean getBottom() {
        return !BOTTOM_SENSOR.get();
    }

    private boolean getTop() {
        return !TOP_SENSOR.get();
    }

    public void setState(HopperState state) {
        actionState = state;
    }

    public HopperState getState() {
        return actionState;
    }

    public void runHopper(double multiplier) {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_POWER*multiplier);
        HOPPER_MOTOR_2.set(Constants.HopperConstants.HOPPER_MOTOR_2_POWER*multiplier);
    }

    public void runHopperOpp() {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, -Constants.HopperConstants.HOPPER_MOTOR_POWER);
        HOPPER_MOTOR_2.set(-Constants.HopperConstants.HOPPER_MOTOR_2_POWER);
    }

    public void stopHopper() {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, 0);
        HOPPER_MOTOR_2.set(0);
    }

    public void resetBallCount() {
        ballCount = 0;
        Log.info("Hopper", "Resetting ball count to " + ballCount);
    }

    public int getBallCount() {
        return ballCount;
    }
}
