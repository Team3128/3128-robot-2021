/** 
 * @author Adham Elarabawy 
 */
package org.team3128.grogu.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.control.AsynchronousPid;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.motion.RamseteController;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.drive.Drive;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.grogu.main.MainGrogu;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class FalconDrive extends Drive {

	public enum DriveState {
		TELEOP, RAMSETECONTROL, TURN, DONE
	}

	private static final FalconDrive instance = new FalconDrive();

	public static FalconDrive getInstance() {
		return instance;
	}

	// private ADXRS450_Gyro gyroSensor;
	public AHRS ahrs;
	// private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon,
	// rightSlaveTalon, rightSlave2Talon;
	private RamseteController autonomousDriver;
	private Trajectory trajectory = null;
	private AsynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;

	private double currentTime;
	private double startTime;
	private double totalTime;

	double prevPositionL = 0;
	double prevPositionR = 0;

	double startTimeControl;
	double endTime = 0;

	public LazyTalonFX leftTalon, rightTalon, leftTalonFollower, rightTalonFollower;

	public double left_setpoint, right_setpoint;

	private FalconDrive() {

		// gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		ahrs = new AHRS(SPI.Port.kMXP);

		//left and right are flipped because the driver wanted to flip the direction of driving.

		rightTalon = new LazyTalonFX(Constants.DriveConstants.RIGHT_DRIVE_FRONT_ID);
		rightTalonFollower = new LazyTalonFX(Constants.DriveConstants.RIGHT_DRIVE_MIDDLE_ID);
		// leftTalonSlave2 = new LazyTalonFX(Constants.LEFT_DRIVE_BACK_ID);

		leftTalon = new LazyTalonFX(Constants.DriveConstants.LEFT_DRIVE_FRONT_ID);
		leftTalonFollower = new LazyTalonFX(Constants.DriveConstants.LEFT_DRIVE_MIDDLE_ID);
		// rightTalonSlave2 = new LazyTalonFX(Constants.RIGHT_DRIVE_BACK_ID);


	


		leftTalon.setInverted(false);
		rightTalon.setInverted(true);
		leftTalonFollower.setInverted(false);
		rightTalonFollower.setInverted(true);
		// leftTalonSlave2.setInverted(false);
		// rightTalonSlave2.setInverted(false);
		leftTalon.setSensorPhase(false);
		rightTalon.setSensorPhase(false);

		configMotors();

		driveState = DriveState.TELEOP;

		turnPID = new AsynchronousPid(1.0, 0, 1.2, 0); // P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveConstants.DRIVE_HIGH_SPEED, -Constants.DriveConstants.DRIVE_HIGH_SPEED);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.DriveConstants.DRIVE_ACCEL_LIMIT);
		turnProfiler = new RateLimiter(100);

		// configHigh();
		configAuto();
	}

	@Override
	public void debug() {
		System.out.println("L enc: " + getLeftDistance() + " velo " + getLeftSpeed());
		System.out.println("R enc: " + getRightDistance() + " velo " + getRightSpeed());
		System.out.println("Gyro: " + getAngle()/* getGyroAngle().getDegrees() */);
	}

	@Override
	public void debugSpeed() {
		System.out.println("L speed " + " actual " + getLeftSpeed());
		System.out.println("R speed " + " actual " + getRightSpeed());

	}

	@Override
	public void setRight() {
		setWheelVelocity(new DriveSignal(40, 0));
	}

	@Override
	public void configAuto() {
		rightTalon.config_kP(0, Constants.DriveConstants.K_AUTO_RIGHT_P);
		rightTalon.config_kD(0, Constants.DriveConstants.K_AUTO_RIGHT_D);
		rightTalon.config_kF(0, Constants.DriveConstants.K_AUTO_RIGHT_F);

		leftTalon.config_kP(0, Constants.DriveConstants.K_AUTO_LEFT_P);
		leftTalon.config_kD(0, Constants.DriveConstants.K_AUTO_LEFT_D);
		leftTalon.config_kF(0, Constants.DriveConstants.K_AUTO_LEFT_F);
	}

	@Override
	public void configHigh() {
		driveMultiplier = Constants.DriveConstants.DRIVE_HIGH_SPEED;
	}

	boolean teleopstart = true;

	@Override
	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}

	@Override
	public void calibrateGyro() {
		// gyroSensor.calibrate();
	}

	@Override
	public void printCurrent() {
		System.out.println(leftTalon);
	}

	@Override
	public void configMotors() {
		leftTalonFollower.follow(leftTalon);
		// leftTalonSlave2.follow(leftTalon);
		rightTalonFollower.follow(rightTalon);
		// rightTalonSlave2.follow(rightTalon);

		leftTalon.setNeutralMode(Constants.DriveConstants.DRIVE_IDLE_MODE);
		rightTalon.setNeutralMode(Constants.DriveConstants.DRIVE_IDLE_MODE);
		leftTalonFollower.setNeutralMode(Constants.DriveConstants.DRIVE_IDLE_MODE);
		rightTalonFollower.setNeutralMode(Constants.DriveConstants.DRIVE_IDLE_MODE);
		// leftTalonSlave2.setIdleMode(IdleMode.kCoast);
		// rightTalonSlave2.setIdleMode(IdleMode.kCoast);
		configAuto();
	}

	@Override
	public void resetMotionProfile() {
		moveProfiler.reset();
	}


	public Pose2d getPose() {
		
		return MainGrogu.ekfPosition;

	}




	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				getLeftSpeed() * Constants.DriveConstants.kDriveInchesPerSecPerNUp100ms * Constants.MechanismConstants.inchesToMeters,
				getRightSpeed() * Constants.DriveConstants.kDriveInchesPerSecPerNUp100ms * Constants.MechanismConstants.inchesToMeters);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {

		//NOTE: THIS METHOD IS NEW AND KIND OF SKETCHY
		//MAY NEED A CLAMP FOR VOLTAGE SO IT DOESNT TRY TO SEND TOO MANY
		//ALSO MAY NEED TO MAKE RIGHT VOLTS NEGATIVE BECAUSE IT GOES IN THE OPPOSITE DIRECTION
		//Log.info("Volts", "Left "+ RobotMath.clamp(leftVolts, -12, 12)+" Right "+RobotMath.clamp(rightVolts, -12, 12));

		//double leftBus = leftTalon.getBusVoltage();
		
		double leftBus = 12;

		double leftPercent = RobotMath.clamp(leftVolts, -leftBus, leftBus)/leftBus; 

		double rightBus = 12;

		//double rightBus = rightTalon.getBusVoltage();
		
		double rightPercent = RobotMath.clamp(rightVolts, -rightBus, rightBus)/rightBus; 
		
		// leftTalon.set(ControlMode.PercentOutput, leftPercent);
		// rightTalon.set(ControlMode.PercentOutput, rightPercent);
		leftTalon.set(ControlMode.PercentOutput, rightPercent);
		rightTalon.set(ControlMode.PercentOutput, leftPercent);

		// Log.info("Voltage Stats", "LB: "+leftBus+" RB: "+rightBus+" LP: "+leftPercent+" RP: "+rightPercent);

		//leftTalon.setVoltage(RobotMath.clamp(leftVolts, -12, 12));
		
		
		//rightTalon.setVoltage(RobotMath.clamp(rightVolts, -12, 12));







	}















	@Override
	public double getAngle() {
		return ahrs.getAngle();
	}

	@Override
	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	@Override
	public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(getAngle());
	}

	@Override
	public double getLeftDistance() {
		return leftTalon.getSelectedSensorPosition(0) * Constants.DriveConstants.kDriveNuToInches;
	}

	@Override
	public double getRightDistance() {
		return rightTalon.getSelectedSensorPosition(0) * Constants.DriveConstants.kDriveNuToInches;
	}

	@Override
	public double getSpeed() {
		return (getLeftSpeed() + getRightSpeed()) / 2;
	}

	@Override
	public double getLeftSpeed() {
		return leftTalon.getSelectedSensorVelocity(0) * Constants.DriveConstants.kDriveInchesPerSecPerNUp100ms;
	}

	@Override
	public double getRightSpeed() {
		return rightTalon.getSelectedSensorVelocity(0) * Constants.DriveConstants.kDriveInchesPerSecPerNUp100ms;
	}

	@Override
	public synchronized void setAutoTrajectory(Trajectory autoTraj, boolean isReversed) {
		this.trajectory = autoTraj;
		totalTime = trajectory.getTotalTimeSeconds();
		autonomousDriver = new RamseteController(1.8, 0.7, isReversed, Constants.AutonomousDriveConstants.TRACK_RADIUS); // 2,0.7
	}

	@Override
	public synchronized void startTrajectory() {
		if (trajectory == null) {
			Log.info("FalconDrive", "FATAL // FAILED TRAJECTORY - NULL TRAJECTORY INPUTTED");
			Log.info("FalconDrive", "Returned to teleop control");
			driveState = DriveState.TELEOP;
		} else {
			//configAuto();
			updateRamseteController(true);
			driveState = DriveState.RAMSETECONTROL;
		}
	}

	@Override
	public void setBrakeState(NeutralMode mode) {
	}

	@Override
	public double getVoltage() {
		return 0;
	}

	@Override
	public void setWheelPower(DriveSignal signal) {
		leftTalon.set(ControlMode.PercentOutput, signal.leftVelocity);
		rightTalon.set(ControlMode.PercentOutput, signal.rightVelocity);
	}

	@Override
	public void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > (Constants.DriveConstants.DRIVE_HIGH_SPEED)
				|| Math.abs(setVelocity.leftVelocity) > (Constants.DriveConstants.DRIVE_HIGH_SPEED)) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DriveConstants.DRIVE_HIGH_SPEED + " !", false);
			return;
		}

		left_setpoint = setVelocity.leftVelocity;
		right_setpoint = setVelocity.rightVelocity;
	}

	/**
	 * Update the motor outputs with the given control values.
	 *
	 * @param joyX     horizontal control input
	 * @param joyY     vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %,
	 *                 0 is 50%, 1.0 is 100%)
	 */
	@Override
	public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}

		double spdL, spdR, pwrL, pwrR;

		if (!fullSpeed) {
			joyY *= .65;
		} else {
			joyY *= 1;
		}

		// scale from 1 to -1 to 1 to 0
		throttle = (throttle + 1) / 2;

		if (throttle < .3) {
			throttle = .3;
		} else if (throttle > .8) {
			throttle = 1;
		}

		joyY *= throttle;
		joyX *= throttle;

		pwrL = Constants.DriveConstants.LEFT_SPEEDSCALAR * RobotMath.clampPosNeg1(joyY - joyX);
		pwrR = Constants.DriveConstants.RIGHT_SPEEDSCALAR * RobotMath.clampPosNeg1(joyY + joyX);

		spdL = Constants.DriveConstants.DRIVE_HIGH_SPEED * pwrL;
		spdR = Constants.DriveConstants.DRIVE_HIGH_SPEED * pwrR;
		// String tempStr = "pwrL=" + String.valueOf(pwrL) + ", pwrR=" + String.valueOf(pwrR) + ", spdL="
		//		+ String.valueOf(spdL) + ", spdR=" + String.valueOf(spdR);
		// Log.info("FalconDrive", tempStr);
		//pwrL
		setWheelPower(new DriveSignal(pwrL, pwrR));
		// setWheelVelocity(new DriveSignal(spdL, spdR));
	}

	@Override
	public void periodic() {
		// velocityController();
		DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
				break;
			case RAMSETECONTROL:
				updateRamseteController(false);
				break;
			case TURN:
				updateTurn();
				break;
		}
	}

	private void velocityController() {
		double ks_sign = 1;

		if (left_setpoint < 0) {
			ks_sign = -1;
		}

		if (left_setpoint == 0) {
			ks_sign = 0;
		}
		double error = left_setpoint - getLeftSpeed();
		double feedforward_left = (Constants.DriveConstants.kS * ks_sign) + (left_setpoint * Constants.DriveConstants.kV); //TODO: add acceleration to this
		double voltage_applied_left = feedforward_left + (Constants.DriveConstants.kP*error);

		ks_sign = 1;

		//the next few conditional statements ensure that the y intercept is pushed to the right direction (in case out path includes going backwards)

		if (right_setpoint < 0) {
			ks_sign = -1;
		}

		if (right_setpoint == 0) {
			ks_sign = 0;
		}


		// applied_voltage = kS + (desired velocity * kV) [we currently ignore acceleration and emit PID which might be a bad assumption]
		error = right_setpoint - getRightSpeed();
		double feedforward_right = (Constants.DriveConstants.kS * ks_sign) + (right_setpoint * Constants.DriveConstants.kV); //TODO: add acceleration to this
		double voltage_applied_right = feedforward_right + (Constants.DriveConstants.kP*error);


		double leftAvaliableVoltage = leftTalon.getBusVoltage();
		double rightAvaliableVoltage = rightTalon.getBusVoltage();

		//Log.info("FalconDrive", "left_voltage = " + String.valueOf(voltage_applied_left) + ", right_voltage = " + String.valueOf(voltage_applied_right));

		double leftPower = voltage_applied_left / leftAvaliableVoltage;
		double rightPower = voltage_applied_right / rightAvaliableVoltage;

		if ((Math.abs(leftPower) > 1) || (Math.abs(rightPower) > 1)) {
			Log.info("Drive", "Tried to set a voltage greater than the avaliable voltage");
			leftPower = RobotMath.clampPosNeg1(leftPower);
			rightPower = RobotMath.clampPosNeg1(rightPower);
		}

		leftTalon.set(ControlMode.PercentOutput, leftPower);
		rightTalon.set(ControlMode.PercentOutput, rightPower);

		endTime = Timer.getFPGATimestamp();
	}

	@Override
	public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
		configHigh();
	}

	/**
	 * Set Velocity PID for both sides of the drivetrain (to the same constants)
	 */
	@Override
	public void setDualVelocityPID(double kP, double kD, double kF) {
		leftTalon.config_kP(0, kP);
		leftTalon.config_kD(0, kD);
		leftTalon.config_kF(0, kF);

		rightTalon.config_kP(0, kP);
		rightTalon.config_kD(0, kD);
		rightTalon.config_kF(0, kF);

		Log.info("[ArcadeDrive]", "Updated Velocity PID values for both sides of the drivetrain to: kP = " + kP
				+ ", kD = " + kD + ", kF = " + kF);
	}

	@Override
	public void setShiftState(boolean state) {
		configHigh();
	}

	
	@Override
	public void resetGyro() {
		ahrs.reset();
		//Log.info("Reset Gyro", " ");
	}

	@Override
	public boolean checkSubsystem() {
		configMotors();
		return true;
	}

	@Override
	synchronized public void stopMovement() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
		setWheelVelocity(new DriveSignal(0, 0));

		driveState = DriveState.TELEOP;
		resetMotionProfile();
	}

	@Override
	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
	}

	@Override
	public void clearStickyFaults() {
	}

	@Override
	public void updateTurn() {
		// TODO Auto-generated method stub

	}

	@Override
	public void updateRamseteController(boolean isStart) {
		// TODO Auto-generated method stub

	}
}
