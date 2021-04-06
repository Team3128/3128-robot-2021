package org.team3128.grogu.subsystems;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
/*  
    Garrison:
    OMG remove unused imports! 
*/
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

public class Hopper implements Subsystem {

    private enum HopperState {
        IDLE,INTAKING,SHOOTING;
    }

    public static final Hopper instance = new Hopper();
    private LazyTalonSRX ARM_MOTOR, HOPPER_MOTOR_1;
    private LazyVictorSPX BRUSH_MOTOR, INTAKE_MOTOR;

    private LazyCANSparkMax HOPPER_MOTOR_2;

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;

    private HopperState actionState = HopperState.IDLE;
    
    private int ballCount;

    private boolean wasTriggeredTop = false;
    private boolean wasTriggeredBottom = false;

    public boolean unshoot = false;
    public boolean intakeShooting = false;

    //TODO: set this boolean to boolean in shooter class 
    private boolean isShooterReady = false;


    /*  
        Garrison:
        This is an incomplete singleton pattern.
        Nothing prevents the code from creating multiple instances.
        I recommend fully implementing the singleton pattern.
        https://www.journaldev.com/1377/java-singleton-design-pattern-best-practices-examples
        I prefer #4 in the link above.  
    */

    public static Hopper getInstance() { 
        return instance;
    }

    public Hopper() {
        configMotors();
        configSensors();
        
        ballCount = 0;
    }

    private void configMotors() {
        ARM_MOTOR = new LazyTalonSRX(Constants.IntakeConstants.ARM_MOTOR_ID);
        BRUSH_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_ID);
        INTAKE_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
        HOPPER_MOTOR_1 = new LazyTalonSRX(Constants.HopperConstants.HOPPER_MOTOR_1_ID);
        HOPPER_MOTOR_2 = new LazyCANSparkMax(Constants.HopperConstants.HOPPER_MOTOR_2_ID, MotorType.kBrushless);


        ARM_MOTOR.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);

    }
    
    private void configSensors() {
       BOTTOM_SENSOR = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID); 
       TOP_SENSOR = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
    }

    @Override
    public void periodic() {
        switch(actionState) {
            case IDLE:
                //stopIntake();
                //stopArm();
                stopHopper();
                if (isShooterReady) {
                    setState(HopperState.SHOOTING);
                    Log.info("Hopper","I don't feel so good (SHOOTER)");
                } else if (getBottom()) {
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
        //TODO: update this variable
        isShooterReady = (Sidekick.getInstance().isReady() && Shooter.getInstance().isReady());
    }

    /*  
        Garrison:
        The logic for increasing or decreasing ball count should be decoupled from the state.
        I would simply have a method that checks and updates ball count every periodic loop.  
    */

    private void intake() {
        Log.info("hopper","intaking");
        if (ballCount >= 2 || getTop()) {
            if (ballCount > 2)
                Log.info("Hopper","oopsie, should not be greater than 3");
            Log.info("Hopper Stomach","FULL!!!!");
            setState(HopperState.IDLE);
       } else {
            runHopper();
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
            runHopper();
            if (!getBottom() && wasTriggeredBottom) {
                intakeShooting = false;
                ballCount++;
                Log.info("Hopper Stomach", "Has eaten 1 ball. Ball count = " + ballCount);
            }
       }

    }

    private void shoot() {
        runHopper();
        Log.info("hopper", "Shooting");
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

    public void runHopper() {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_POWER);
        HOPPER_MOTOR_2.set(Constants.HopperConstants.HOPPER_MOTOR_2_POWER);
    }

    public void stopHopper() {
        HOPPER_MOTOR_1.set(ControlMode.PercentOutput, 0);
        HOPPER_MOTOR_2.set(0);
    }

    public void runIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void stopIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, 0);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmDown() {
        ARM_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmUp() {
        ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void stopArm() {
        ARM_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void resetBallCount() {
        ballCount = 0;
        Log.info("Hopper", "Resetting ball count to " + ballCount);
    }

    public int getBallCount() {
        return ballCount;
    }
}
