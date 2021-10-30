package org.team3128.grogu.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.sim.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Shooter extends PIDSubsystem {
    public enum ShooterState {
        OFF(0),
        LONG_RANGE(5500), // long range shooting
        MID_RANGE(4800), //4800 actual setpoint  // mid range shooting
        SHORT_RANGE(3500),
        GREEN(1200),
        YELLOW(5000),
        BLUE(3330),
        RED(3333),
        ; // short range shooting 3700

        public double shooterRPM;

        private ShooterState(double RPM) {
            this.shooterRPM = RPM;
        }
    }

    public static final Shooter instance = new Shooter();
    public static BaseTalon LEFT_SHOOTER;
    public static BaseTalon RIGHT_SHOOTER;
    public static TalonSRXSimCollection LEFT_SHOOTERSim;
    public static TalonSRXSimCollection RIGHT_SHOOTERSim;

    public static DifferentialDrivetrainSim driveSim;

    public static boolean DEBUG = true;
    double current = 0;
    double error = 0;
    public double output = 0;
    double accumulator = 0;
    double prevError = 0;
    public boolean isAligned = false;

    double value = 0, preValue = 0, time = 0, preTime = 0;

    int plateauCount = 0;
    private double thresholdPercent = Constants.ShooterConstants.RPM_THRESHOLD_PERCENT;

    // private StateTracker stateTracker = StateTracker.getInstance();
    public ShooterState SHOOTER_STATE = ShooterState.MID_RANGE;

    private Shooter() {

        super(new PIDController(Constants.ShooterConstants.SHOOTER_PID.kP, Constants.ShooterConstants.SHOOTER_PID.kI, Constants.ShooterConstants.SHOOTER_PID.kD));
        getController().setTolerance(thresholdPercent * 4800);
        //.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
        

        configMotors();
        configEncoders();
        preTime = RobotController.getFPGATime()/ 1e6;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public boolean isPlateaued() {
        return (plateauCount >= Constants.ShooterConstants.PLATEAU_COUNT);
    }

    private void configMotors() {
        if(Robot.isReal()){
            LEFT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
            RIGHT_SHOOTER = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
            if (DEBUG) {
                Log.info("Shooter", "Config motors");
            }
        }else{
            LEFT_SHOOTER = new TalonSRX(Constants.ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
            RIGHT_SHOOTER = new TalonSRX(Constants.ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
            LEFT_SHOOTERSim = ((TalonSRX)LEFT_SHOOTER).getSimCollection();
            RIGHT_SHOOTERSim = ((TalonSRX)RIGHT_SHOOTER).getSimCollection();

            driveSim = new DifferentialDrivetrainSim(
                DCMotor.getFalcon500(2), 
                0, 
                0, 
                0, 
                0, 
                0,
                VecBuilder.fill(0,0,0.0001,0.1,0.1,0.005,0.005)
            );
        }
    }

    private void configEncoders() {
        // SHOOTER_ENCODER = LEFT_SHOOTER.getEncoder();
        // if (DEBUG) {
        //     Log.info("Shooter", "Config encoders");
        // }
    }

    public static Shooter getInstance() {
        return instance;
    }

    @Override
    public double getMeasurement() {
        //Log.info("shooter", "getting measurement");
        return LEFT_SHOOTER.getSelectedSensorVelocity(0) * 10 * 60 / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        double voltageOutput = shooterFeedForward(setpoint) + output;
        double voltage = RobotController.getBatteryVoltage(); // TODO: investigate bus voltage

        output = voltageOutput / 12;//voltage

        //Log.info("Shooter", "using output");

        value = getMeasurement();
        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPercent < Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX) {
            thresholdPercent += ((time - preTime) * ((Constants.ShooterConstants.RPM_THRESHOLD_PERCENT_MAX - Constants.ShooterConstants.RPM_THRESHOLD_PERCENT)) / Constants.ShooterConstants.TIME_TO_MAX_THRESHOLD);
        }
        
        double accel = (value - preValue) / (time - preTime);

        // Log.info("Shooter",getMeasurement()+" RPM");

        if (atSetpoint() && (setpoint != 0)) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }

        preValue = value;
        preTime = time;

        if (output > 1) {
            // Log.info("SHOOTER",
            // "WARNING: Tried to set power above available voltage! Saturation limit SHOULD
            // take care of this ");
            output = 1;
        } else if (output < -1) {
            // Log.info("SHOOTER",
            // "WARNING: Tried to set power above available voltage! Saturation limit SHOULD
            // take care of this ");
            output = -1;
        }

        if(setpoint == 0) {
            output = 0;
        }

        // if (accel > Constants.SHOOTER_MAX_ACCELERATION)
        //     output = output / (accel / Constants.SHOOTER_MAX_ACCELERATION);

        // if (setpoint != 0) {
        //     Log.info("Shooter", "shooter setpoint = " + setpoint);
        // }

        LEFT_SHOOTER.set(ControlMode.PercentOutput, output);
        RIGHT_SHOOTER.set(ControlMode.PercentOutput, -output);
    }

    // overriding PIDController setSetpoint() to set the tolerance as well
    // public void setSetpoint(double passedSetpoint) {
    //     plateauCount = 0;
    //     super.setSetpoint(passedSetpoint);
    //     getController().setTolerance(thresholdPercent * passedSetpoint);
    //     //Log.info("Shooter", "Set setpoint to" + String.valueOf(setpoint));
    // }

    public void setSetpointAndTolerance(double passedSetpoint) {
        plateauCount = 0;
        super.setSetpoint(passedSetpoint);
        getController().setTolerance(thresholdPercent * passedSetpoint);
        //Log.info("Shooter", "Set setpoint to" + String.valueOf(setpoint));
    }




    public void setState(ShooterState shooterState) {
        SHOOTER_STATE = shooterState;
        //setSetpoint(shooterState.shooterRPM);
    }

    public void shoot() {
        setSetpointAndTolerance(SHOOTER_STATE.shooterRPM);
    }

    public void counterShoot() {
        setSetpoint(0);
    }

    @Override
    public void simulationPeriodic() {
        Log.info("Shooter", "running sim periodic");
        
        driveSim.setInputs(LEFT_SHOOTER.getMotorOutputVoltage(), RIGHT_SHOOTER.getMotorOutputVoltage());
        driveSim.update(0.02);

        System.out.println("herepog");
        System.out.println(
            (int)driveSim.getLeftPositionMeters() + " " +
            LEFT_SHOOTER.getMotorOutputVoltage() + " " +
            RobotController.getBatteryVoltage()
        );

        LEFT_SHOOTERSim.setQuadratureRawPosition((int)driveSim.getLeftPositionMeters());
        LEFT_SHOOTERSim.setQuadratureVelocity((int)driveSim.getLeftVelocityMetersPerSecond());
        RIGHT_SHOOTERSim.setQuadratureRawPosition((int)driveSim.getRightPositionMeters());
        RIGHT_SHOOTERSim.setQuadratureVelocity((int)driveSim.getRightVelocityMetersPerSecond());

        LEFT_SHOOTERSim.setBusVoltage(RobotController.getBatteryVoltage());
        RIGHT_SHOOTERSim.setBusVoltage(RobotController.getBatteryVoltage());
    }



    public double shooterFeedForward(double desiredSetpoint) {
        //double ff = (0.00211 * desiredSetpoint) - 2; // 0.051
        // feed forward: 0.0019
        double ff = ((0.0019) * desiredSetpoint);//0.00168//0.00170 // 0.00188*x //0.00147x - 0.2; // 0
        if (getSetpoint() != 0) {
            return ff;
        } else {
            return 0;
        }
    }

    // public double getRPMFromDistance() {
    //     return stateTracker.getState().targetShooterState.shooterRPM;
    // }

    // public boolean isReady() {
    //     return (plateauCount > Constants.PLATEAU_COUNT);
    // }

    // public void queue(){
    //     setState(stateTracker.getState().targetShooterState);
    // }

    public ShooterState getState() {
        return SHOOTER_STATE;
    }

    public boolean isReady() {
        if (atSetpoint())
            Log.info("Shooter","at Setpoint");
        if (isAligned)
            Log.info("Shooter","is Aligned");
        return ((isAligned || SHOOTER_STATE == ShooterState.GREEN) && isPlateaued());
    }
}
