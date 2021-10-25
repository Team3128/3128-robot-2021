package org.team3128.grogu.commands;

import java.util.HashSet;
import java.util.Set;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.grogu.subsystems.Constants;
import org.team3128.grogu.subsystems.FalconDrive;
import org.team3128.grogu.subsystems.Hopper;
import org.team3128.grogu.subsystems.Intake;
import org.team3128.grogu.subsystems.StateTracker;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CmdAlignShoot implements Command {
    FalconDrive drive;
    Hopper hopper;
    StateTracker stateTracker;

    boolean gotDistance = false;


    Limelight limelight;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID;

    private double goalHorizontalOffset;

    private double currentHorizontalOffset;

    private double currentError, previousError;
    private double currentTime, previousTime, initialTime;

    private double feedbackPower;

    private double leftPower, rightPower;

    private double desiredRPM;
    private double effective_distance;

    private Set<Subsystem> requirements;

    private Command hopperShoot, organize;

    int targetFoundCount;
    int plateauCount;

    int numBallsShot;
    int numBallsToShoot;

    private double txThreshold = Constants.VisionConstants.TX_THRESHOLD;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK; // , BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdAlignShoot(Limelight limelight, DriveCommandRunning cmdRunning, double goalHorizontalOffset, int numBallsToShoot) {
        this.hopper = Hopper.getInstance();
        this.drive = FalconDrive.getInstance();
        this.stateTracker = StateTracker.getInstance();

        this.requirements = new HashSet<Subsystem>();
        this.requirements.add(drive);
        this.requirements.add(hopper);
        
        this.limelight = limelight;
        this.visionPID = Constants.VisionConstants.VISION_PID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.numBallsToShoot = numBallsToShoot;
        
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        limelight.setLEDMode(LEDMode.ON);
        cmdRunning.isRunning = false;
        plateauCount = 0;

        stateTracker.enterShoot();

        Log.info("CmdAlignShoot", "initialized limelight, aren't I cool!");
        hopper.ballCount = 3;
        initialTime = RobotController.getFPGATime() / 1e6;
    }

    @Override
    public void execute() {
        //Log.info("CmdAlignShoot", "Running one loop of execute");
        currentTime = RobotController.getFPGATime() / 1e6;

        switch (aimState) {
            case SEARCHING:
                NarwhalDashboard.put("align_status", "searching");
                Log.info("CmdAlignShoot", "Searching...");
                if (limelight.hasValidTarget()) {
                    targetFoundCount += 1;
                } else {
                    targetFoundCount = 0;
                }

                if (targetFoundCount > 5) {
                    Log.info("CmdAlignShoot", "Target found.");
                    Log.info("CmdAlignShoot", "Switching to FEEDBACK...");
                    LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                    SmartDashboard.putNumber("ty", initData.ty());

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionConstants.SAMPLE_RATE);//5);

                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                NarwhalDashboard.put("align_status", "feedback");
                cmdRunning.isRunning = false;
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAlignShoot", "No valid target.");
                    Log.info("CmdAlignShoot", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                } else {

                    if (!gotDistance) {
                        LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                        SmartDashboard.putNumber("ty", initData.ty());


                        gotDistance = true;
                    }

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionConstants.SAMPLE_RATE);

                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    if (txThreshold < Constants.VisionConstants.TX_THRESHOLD_MAX) {
                        Log.info("CmdAlignShoot", String.valueOf(txThreshold));
                        //Log.info("CmdAlignShootagain", String.valueOf(currentTime - previousTime));
                        //Log.info("CmdAlignShootEntire", String.valueOf((currentTime - previousTime) * ((Constants.VisionConstants.TX_THRESHOLD_MAX - Constants.VisionConstants.TX_THRESHOLD)) / Constants.VisionConstants.TIME_TO_MAX_THRESHOLD));
                        txThreshold += ((currentTime - previousTime) * ((Constants.VisionConstants.TX_THRESHOLD_MAX - Constants.VisionConstants.TX_THRESHOLD)) / Constants.VisionConstants.TIME_TO_MAX_THRESHOLD);
                    }

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal
                     * offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentError;
                    feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                    leftPower = RobotMath.clamp(-feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(feedbackPower, -1, 1);

                    double leftSpeed = leftPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    double rightSpeed = rightPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    
                    drive.setWheelPower(new DriveSignal(leftPower, rightPower));
                    previousError = currentError;
                }
                if ((Math.abs(currentError) < (txThreshold * Constants.VisionConstants.TX_THRESHOLD))) {
                    plateauCount++;
                    if (plateauCount > 10) {
                        stateTracker.setAligned(true);
                        Log.info("Cmd Align Shoot","SHOOTY TIME!!!");
                    }
                } else {
                    stateTracker.setAligned(false);
                    plateauCount = 0;
                }
                break;
        }

        previousTime = currentTime;
    }

    @Override
    public boolean isFinished() {
        // if (hopper.getBallCount() == 0|| numBallsShot >= numBallsToShoot) {
        // return true;
        // } else {
        // return false;
        // }
        return (stateTracker.getAligned() && hopper.getBallCount() == 0) || (currentTime - initialTime > Constants.VisionConstants.AUTO_ALIGN_TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        Log.info("CmdAlignShoot", "end");
        limelight.setLEDMode(LEDMode.OFF);
        drive.stopMovement();
        stateTracker.exitShoot();
        // intake.stopArm();
        //hopper.runIntake();

        Log.info("CmdAlignShoot", "Command Finished.");
        if (interrupted)
            Log.info("CmdAlignShoot", "Command interru-");
        //hopper.setAction(Hopper.ActionState.ORGANIZING);
    }
}