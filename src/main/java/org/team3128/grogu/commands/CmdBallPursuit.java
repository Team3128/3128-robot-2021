package org.team3128.grogu.commands;

import java.util.HashSet;
import java.util.Set;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.Pipeline;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.grogu.subsystems.FalconDrive;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CmdBallPursuit implements Command {
    FalconDrive drive;
    AHRS gyro;

    DriveCalibrationUtility dcu;

    Limelight ballLimelight;

    // private final double FEED_FORWARD_POWER = 0.55;
    // private final double MINIMUM_POWER = 0.1;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 3;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID, blindPID;

    private double multiplier;

    private double goalHorizontalOffset;
    private double targetHeight;

    private double currentHorizontalOffset;
    private double previousVerticalAngle, approximateDistance;

    private double currentAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftVel, rightVel;

    private double leftPower, rightPower;

    private double blindThreshold;

    private boolean isLowHatch;

    private Set<Subsystem> requirements;

    int targetFoundCount;
    int plateauReachedCount;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdBallPursuit(AHRS gyro, Limelight ballLimelight,
            DriveCommandRunning cmdRunning, double targetHeight, PIDConstants visionPID, double goalHorizontalOffset,
            double decelerationStartDistance, double decelerationEndDistance, PIDConstants blindPID,
            double blindThreshold) {

        this.gyro = gyro;
        this.ballLimelight = ballLimelight;
        this.visionPID = visionPID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.targetHeight = targetHeight;

        this.decelerationStartDistance = decelerationStartDistance;
        this.decelerationEndDistance = decelerationEndDistance;

        this.blindPID = blindPID;
        this.blindThreshold = blindThreshold;

        this.requirements = new HashSet<Subsystem>();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        drive = FalconDrive.getInstance();
        //dcu = DriveCalibrationUtility.getInstance();

        ballLimelight.setLEDMode(LEDMode.OFF);
        ballLimelight.setPipeline(Pipeline.GRIP);
        
        cmdRunning.isRunning = false;
    }

    @Override
    public void execute() {
        switch (aimState) {
        case SEARCHING:
            NarwhalDashboard.put("align_status", "searching");
            if (ballLimelight.hasValidTarget()) {
                targetFoundCount += 1;
            } else {
                targetFoundCount = 0;
            }

            if (targetFoundCount > 5) {
                Log.info("CmdAutoAim", "Target found.");
                Log.info("CmdAutoAim", "Switching to FEEDBACK...");

                drive.setWheelPower(new DriveSignal(0.8*visionPID.kF, 0.8*visionPID.kF));

                currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                previousTime = RobotController.getFPGATime();
                previousError = goalHorizontalOffset - currentHorizontalOffset;

                cmdRunning.isRunning = true;

                aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
            }

            break;

        case FEEDBACK:
            NarwhalDashboard.put("align_status", "feedback");
            Log.info("CmdBallPursuit","prevVertAngle: "+ previousVerticalAngle +"\nballLimelight.hasValidTarget(): "+ ballLimelight.hasValidTarget());
            if (!ballLimelight.hasValidTarget()) {
                Log.info("CmdAutoAim", "No valid target.");
                if (/*(ballLimelight.cameraAngle > 0 ? 1 : -1) **/ previousVerticalAngle > blindThreshold) {
                    Log.info("CmdAutoAim", "Switching to BLIND...");

                    gyro.reset();
                    aimState = HorizontalOffsetFeedbackDriveState.BLIND;
                } else {
                    Log.info("CmdAutoAim", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                    cmdRunning.isRunning = false;
                }
            } else {
                currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                currentTime = RobotController.getFPGATime();
                currentError = goalHorizontalOffset - currentHorizontalOffset;

                /**
                 * PID feedback loop for the left and right powers based on the horizontal
                 * offset errors.
                 */
                feedbackPower = 0;

                feedbackPower += visionPID.kP * currentError;
                feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                leftPower = RobotMath.clamp(visionPID.kF - feedbackPower, -1, 1);
                rightPower = RobotMath.clamp(visionPID.kF + feedbackPower, -1, 1);

                previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2);
                approximateDistance = ballLimelight.calculateYPrimeFromTY(previousVerticalAngle, targetHeight);

                multiplier = 1.0 - (1.0 - blindPID.kF / visionPID.kF)
                        * RobotMath.clamp((decelerationStartDistance - approximateDistance)
                                / (decelerationStartDistance - decelerationEndDistance), 0.0, 1.0);

                drive.setWheelPower(new DriveSignal(0.7*multiplier * leftPower, 0.7*multiplier * rightPower));
                previousTime = currentTime;
                previousError = currentError;
            }

            break;

        case BLIND:
            NarwhalDashboard.put("align_status", "blind");

            currentAngle = gyro.getAngle();

            currentTime = RobotController.getFPGATime() / 1000000.0;
            currentError = -currentAngle;

            /**
             * PID feedback loop for the left and right powers based on the gyro angle
             */
            feedbackPower = 0;

            feedbackPower += blindPID.kP * currentError;
            feedbackPower += blindPID.kD * (currentError - previousError) / (currentTime - previousTime);

            rightPower = RobotMath.clamp(blindPID.kF - feedbackPower, -1, 1);
            leftPower = RobotMath.clamp(blindPID.kF + feedbackPower, -1, 1);

            Log.info("CmdAutoAim", "L: " + leftPower + "; R: " + rightPower);

            drive.setWheelPower(new DriveSignal(0.7*leftPower, 0.7*rightPower)); //TODO: remove the 0.7's once testing is done

            previousTime = currentTime;
            previousError = currentError;
            Log.info("CmdAutoAim", "Error:" + currentError);

            break;
        }
    }

    @Override
    public boolean isFinished() {
        if (aimState == HorizontalOffsetFeedbackDriveState.BLIND) {
            leftVel = Math.abs(drive.getLeftSpeed());
            rightVel = Math.abs(drive.getRightSpeed());

            if (leftVel < VELOCITY_THRESHOLD && rightVel < VELOCITY_THRESHOLD) {
                plateauReachedCount += 1;
            } else {
                plateauReachedCount = 0;
            }
            Log.info("CmdBallPursuit","plateau: "+plateauReachedCount);

            if (plateauReachedCount >= VELOCITY_PLATEAU_COUNT) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMovement();
        
        NarwhalDashboard.put("align_status", "blind");

        cmdRunning.isRunning = false;

        Log.info("CmdBallPursuit", "Command Finished.");
    }
}