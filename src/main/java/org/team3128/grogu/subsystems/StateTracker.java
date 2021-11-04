package org.team3128.grogu.subsystems;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateTracker implements Subsystem {

    public static StateTracker instance = new StateTracker();

    Shooter shooter;
    Sidekick sidekick;
    Hopper2 hopper;
    Intake intake;
    public boolean isShooterReady;

    private boolean isAligned = false;

    public StateTracker() {
        shooter = Shooter.getInstance();
        sidekick = Sidekick.getInstance();
        hopper = Hopper2.getInstance();
        intake = Intake.getInstance();
    }

    public static StateTracker getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        isShooterReady = shooter.isReady() && sidekick.isReady() && isAligned;
        Log.info("StateTracker", "shooter.isReady(): " + shooter.isReady());
        Log.info("StateTracker", "sidekick.isReady(): " + sidekick.isReady());
        hopper.setShooting(isShooterReady);
    }

    public void enterShoot() {
        sidekick.shoot();
        shooter.shoot();
    }

    public void exitShoot() {
        intake.stopIntake();
        sidekick.counterShoot();
        shooter.counterShoot();
        isAligned = false;
    }

    public void setAligned(boolean isAligned) {
        this.isAligned = isAligned;
    }

    public boolean getAligned() {
        return isAligned;
    }
}
