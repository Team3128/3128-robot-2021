package org.team3128.grogu.subsystems;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.test_suite.CanDevices;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import org.team3128.testbench.subsystems.Constants;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;


public class Climber implements Subsystem {
    private LazyTalonSRX CLIMBER_MOTOR_1, CLIMBER_MOTOR_2;
    public static final Climber instance = new Climber();
    
    
    public static Climber getInstance() {
        return instance;
    }

    public Climber() {
        configMotors();
        configEncoders();
    }

    private void configMotors() {
        CLIMBER_MOTOR_1 = new LazyTalonSRX(Constants.ClimberConstants.CLIMBER_MOTOR_1_ID);
        CLIMBER_MOTOR_2 = new LazyTalonSRX(Constants.ClimberConstants.CLIMBER_MOTOR_2_ID);
        // CLIMBER_MOTOR_2.set(ControlMode.Follower, Constants.ClimberConstants.CLIMBER_MOTOR_1_ID);

        CLIMBER_MOTOR_1.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
        CLIMBER_MOTOR_2.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
    }

    private void configEncoders() {
    }

    public void moveClimberUp() {
        CLIMBER_MOTOR_1.set(ControlMode.PercentOutput, Constants.ClimberConstants.CLIMBER_MOTOR_POWER);
        CLIMBER_MOTOR_2.set(ControlMode.PercentOutput, Constants.ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void moveClimberDown() {
        CLIMBER_MOTOR_1.set(ControlMode.PercentOutput, -Constants.ClimberConstants.CLIMBER_MOTOR_POWER);
        CLIMBER_MOTOR_2.set(ControlMode.PercentOutput, -Constants.ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void stopClimber() {
        CLIMBER_MOTOR_1.set(ControlMode.PercentOutput, 0);
        CLIMBER_MOTOR_2.set(ControlMode.PercentOutput, 0);
    }
}