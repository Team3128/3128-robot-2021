package org.team3128.grogu.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.grogu.subsystems.*;
import org.team3128.grogu.subsystems.PathFinding;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSimple extends SequentialCommandGroup {

    public AutoSimple(Limelight shooterLimelight, DriveCommandRunning cmdRunning, double goalHorizontalOffset, PathFinding m_robotContainer, FalconDrive mRobotDrive, Intake intake) {       
        addCommands(
            new CmdAlignShoot(shooterLimelight, cmdRunning, goalHorizontalOffset, 3),
            new InstantCommand(() -> intake.runIntake()),
            m_robotContainer.getAutonomousCommandSimple(mRobotDrive),
            new InstantCommand(() -> intake.stopIntake())
            );
    }
}