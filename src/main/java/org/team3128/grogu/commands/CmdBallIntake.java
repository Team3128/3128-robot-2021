package org.team3128.grogu.commands;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.grogu.subsystems.Constants;
import org.team3128.grogu.subsystems.FalconDrive;
import org.team3128.grogu.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdBallIntake extends SequentialCommandGroup {

    public CmdBallIntake(FalconDrive drive, Hopper hopper, AHRS ahrs, Limelight ballLimelight, DriveCommandRunning driveCmdRunning) {       
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> hopper.runIntake()),
                new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,20 * Angle.DEGREES)
                    //new CmdStreamUpdate(bottomLimelight, topLimelight, useBottom)
            ),
            new InstantCommand(() -> hopper.stopIntake())
        );
    }
}