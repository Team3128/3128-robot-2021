/*
    Garrison:
    I'm using this file as the main entry point for lots of comments.
    Most of what I suggest is probably simple refactoring.
    There might be a couple of structural changes that I recommend, also.
    There may be comments in other files that can easily be found in the changelog. 

    My philosophy for code is that it should be READABLE, first and foremost. 
    This is hard to define but basically it boils down to anyone being able to 
    read and understand your code easily. They should not have to become familiar 
    with too much of the underlying interfaces to understand what is going on.
    Basically a random developer should be able to read and understand the code
    within an hour or two and be able to modify or implement logic.
*/


package org.team3128.grogu.main;

/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.common.utility.Log;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.grogu.subsystems.*;
import org.team3128.grogu.commands.*;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
/*
    Garrison: 
    Remove unused imports that clutter the code.
*/
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class MainGrogu extends NarwhalRobot {

    /*  
        Garrison:
        This needs to be refactored. 
        A class should DO something. 
        This either needs to be a struct or should just be a 
        boolean as there is only one boolean value stored in 
        the class. 
    */
    private DriveCommandRunning driveCmdRunning;

    // public StateTracker stateTracker = StateTracker.getInstance();

 
    // RobotTracker robotTracker = RobotTracker.getInstance();

    /*
        Garrison: 
        What does this do?
        It has no references.
    */
    ExecutorService executor = Executors.newFixedThreadPool(6);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    /*
        Garrison: 
        This also has no references. 
    */
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    /*
        Garrison: 
        Recommend naming this ahrsGyro so it is clear what it is.
    */
    public AHRS ahrs;
    /*
        Garrison: 
        Why is this static?
        It also has no references...
        Perhaps stuff that is on every robot should be setup automatically?
    */
    public static PowerDistributionPanel pdp;

    /*  
        Garrison:
        These also have no references but I assume they were used last year.
        I would suggest better names to more easily identify what they are.
        limelightNetworkTable, for example.
    */
    public NetworkTable table;
    public NetworkTable limelightTable;

    /*  
        Garrison:
        No references on this one...
    */
    public double startTime = 0;

    /*  
        Garrison:
        This should have a better name. 
        Also probably not the best way to do this but convenient if you are changing often.
    */
    public int reverse = 1;

    /*  
        Garrison:
        None of these have references.
        Removing these will help with code clarity.
    */
    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";
    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight shooterLimelight, ballLimelight;
    /*  
        Garrison:
        The naming on these is bad...
        Also not referenced...
    */
    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public FalconDrive drive = FalconDrive.getInstance();
    public Hopper hopper = Hopper.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Sidekick sidekick = Sidekick.getInstance();

    public CmdAlignShoot alignCmd;

    @Override
    protected void constructHardware() {

        //shooterLimelight.setLEDMode(LEDMode.OFF);
        //ballLimelight.setLEDMode(LEDMode.OFF);
        
        
        driveCmdRunning = new DriveCommandRunning(); 

        ahrs = drive.ahrs;

        //hopper.enable();

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

        // initialization of limelights

        shooterLimelight = new Limelight("limelight-shooter", 26.0, 0, 0, 30);
        ballLimelight = new Limelight("limelight-c", Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_HEIGHT,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        drive.resetGyro();

        /*
            Garrison:
            This should probably be called inside of the hopper class...
            Seems like a weird requirement for WPILIB to have to register this.
            Not sure I would have a subsystem that wasn't registered and running periodic updates.
            I see down below that you are using the listeners to manage logic AND functions of the shooter.
            This is not great from a decoupling perspective. The functions and logic should be handled in the shooter class.
            The listeners should simply set flags on the subsystems. Such as shooter.SetState(Shooter.ShooterState.Shooting).
            The initialization states should also be handled within the class of the subsystem.
            You're basically doing too much outside of the subsystem which can make it hard to figure out what is going on.
        */
        hopper.register();

        shooter.enable();
        shooter.setSetpoint(0);
        sidekick.enable();
        sidekick.setState(Sidekick.ShooterState.DEFAULT);

        shooter.setState(Shooter.ShooterState.MID_RANGE);
        alignCmd = new CmdAlignShoot(shooterLimelight, driveCmdRunning, 0, 26);
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl( ControllerExtreme3D.TRIGGER, "Intake");

        listenerRight.nameControl(new Button(10), "MoveArmDown");
        listenerRight.nameControl(new Button(8), "MoveArmUp");

        listenerRight.nameControl(new Button(2), "Shoot");

        listenerRight.nameControl(new Button(7), "SetOverYonder");
        listenerRight.nameControl(new Button(9), "SetMiddling");
        listenerRight.nameControl(new Button(11), "SetIntimate");

        listenerRight.nameControl(new Button(12), "ResetBallCount");
        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "REVERSE");

        /*
            Garrison:
            All of this logic should be moved into the subsystems.
        */

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = 0.4  * listenerRight.getAxis("MoveTurn"); //-0.5
                double vert = -1.0 * reverse * listenerRight.getAxis("MoveForwards"); //-1.0
                double throttle = -1.0 * listenerRight.getAxis("Throttle"); // -1.0

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        listenerRight.addButtonDownListener("Intake", () -> {
            hopper.runIntake();
            Log.info("Joystick","Button 3 pressed");
        });

        listenerRight.addButtonUpListener("Intake", () -> {
            hopper.stopIntake();
            Log.info("Joystick","Button 3 unpressed");
        });

        listenerRight.addButtonDownListener("Shoot", () -> {
            //sidekick.setState(Sidekick.ShooterState.MID_RANGE);
            sidekick.shoot();
            shooter.shoot();
            scheduler.schedule(alignCmd);
            Log.info("Joystick","Button 4 pressed");
        });

        listenerRight.addButtonUpListener("Shoot", () -> {
            //sidekick.setState(Sidekick.ShooterState.OFF);
            sidekick.counterShoot();
            shooter.counterShoot();
            hopper.unshoot = true;
            alignCmd.cancel();
            shooter.setSetpoint(0);
            driveCmdRunning.isRunning = true;
            shooter.isAligned = false;
            Log.info("Joystick","Button 4 unpressed");
        });

        /*
            Garrison:
            These are good. The logic is handled in the susbsystem.
        */

        listenerRight.addButtonDownListener("MoveArmDown", () -> {
            hopper.moveArmDown();
        });

        listenerRight.addButtonUpListener("MoveArmDown", () -> {
            hopper.stopArm();
        });

        listenerRight.addButtonDownListener("MoveArmUp", () -> {
            hopper.moveArmUp();
        });

        listenerRight.addButtonUpListener("MoveArmUp", () -> {
            hopper.stopArm();
        });

        listenerRight.addButtonDownListener("ResetBallCount", () -> {
            hopper.resetBallCount();
        });

        /*
            Garrison:
            Move logic into subsystem.
        */

        listenerLeft.addButtonDownListener("REVERSE", () -> {
            reverse *= -1;
        });

        /*
            Garrison:
            This is good, we are setting states but not handling the logic.
        */

        listenerRight.addButtonDownListener("SetOverYonder", () -> {
            shooter.setState(Shooter.ShooterState.LONG_RANGE);
        });
        listenerRight.addButtonDownListener("SetMiddling", () -> {
            shooter.setState(Shooter.ShooterState.MID_RANGE);
        });
        listenerRight.addButtonDownListener("SetIntimate", () -> {
            shooter.setState(Shooter.ShooterState.SHORT_RANGE);
        });

    }

    @Override
    protected void teleopPeriodic() {
    }


    /*
        Garrison:
        These should be moved into the updateDashboard method as they are not used outside of it.
    */

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentRightSpeed;
    double currentSpeed;
    double currentDistance;
    

    @Override
    protected void updateDashboard() {
        // SmartDashboard.putString("hopper update count", String.valueOf(//hopper.hopper_update_count));
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());

        currentLeftSpeed = drive.getLeftSpeed();
        currentRightSpeed = drive.getRightSpeed();

        currentSpeed = drive.getSpeed();
    

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

    }

    @Override
    protected void teleopInit() {
        /*
            Garrison:
            I would move this logic into the shooter.
            You may have multiple things that need to be initialized 
            for the shooter and it is best done there.
        */
        shooterLimelight.setLEDMode(LEDMode.OFF);
        Log.info("MainGrogu", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        /*
            Garrison:
            Again, I would handle this in the drive "subsystem".
            something like drive.autonomousInit().
            Might have this for every subsystem.
            Perhaps a list of Subsystem objects and call an Init funciton on all of them.
        */
        drive.resetGyro();
    }

    @Override
    protected void disabledInit() {
        /*
            Garrison: 
            Move to shooter subsystem.
        */
        shooterLimelight.setLEDMode(LEDMode.OFF);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGrogu::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}