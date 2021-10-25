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

public class Hopper2 implements Subsystem {

    public static final Hopper2 instance = new Hopper2();
    private LazyTalonSRX HOPPER_MOTOR_1;
    private LazyCANSparkMax HOPPER_MOTOR_2;

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;
    private boolean shooting;
    

    public static Hopper2 getInstance() { 
        return instance;
    }

    public Hopper2() {
        configMotors();
        configSensors();
        
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
        if (shooting) {
            runHopper(1);
        }
        else {
            if (getBottom() && !getTop()) {
                runHopper(1);
            }          
            else {
                stopHopper();
            }  
        }

    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    public boolean getShooting() {
        return shooting;
    }

    /*private void intake() {
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
    */

    /*private void intakeShoot() {
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
    */

    /*
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
    */

    private boolean getBottom() {
        return !BOTTOM_SENSOR.get();
    }

    private boolean getTop() {
        return !TOP_SENSOR.get();
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
}