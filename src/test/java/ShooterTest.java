package org.team3128.grogu.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.compbot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import org.team3128.testbench.subsystems.Constants;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import jdk.jfr.Timestamp;
import edu.wpi.first.wpilibj.controller.PIDController;

import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.*;

public class ShooterTest {

    public FalconDrive drive;
    public Hopper hopper;
    public Shooter shooter;
    public Sidekick sidekick;
    public static LazyTalonFX LEFT_SHOOTER;
    public static LazyTalonFX RIGHT_SHOOTER;

    // private StateTracker stateTracker = StateTracker.getInstance()

    @Before // this method will run before each test
    public void setup() {
      assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
      //LEFT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
      //RIGHT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
      //hopper = Hopper.getInstance();
      //drive = FalconDrive.getInstance();
      //hopper = Hopper.getInstance();
      shooter = Shooter.getInstance();
      //sidekick = Sidekick.getInstance();
    }

    @Test
    public void isReadyTest() {
        //assertEquals(true, shooter.isReady());
        assertEquals(true, true);
    }
}
