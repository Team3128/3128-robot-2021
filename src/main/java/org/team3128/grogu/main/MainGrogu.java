package org.team3128.grogu.main;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.grogu.commands.AutoLessSimple;
import org.team3128.grogu.commands.AutoSimple;
import org.team3128.grogu.commands.CmdAlignShootTeleop;
import org.team3128.grogu.commands.CmdBallIntake;
import org.team3128.grogu.commands.CmdBallPursuit;
import org.team3128.grogu.subsystems.Constants;
import org.team3128.grogu.subsystems.EKF;
import org.team3128.grogu.subsystems.FalconDrive;
import org.team3128.grogu.subsystems.Hopper;
import org.team3128.grogu.subsystems.Intake;
import org.team3128.grogu.subsystems.PathFinding;
import org.team3128.grogu.subsystems.Shooter;
import org.team3128.grogu.subsystems.Sidekick;
import org.team3128.grogu.subsystems.StateTracker;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class MainGrogu extends NarwhalRobot {

    private DriveCommandRunning driveCmdRunning;

    private CmdBallIntake cmdBallIntake;
    private CmdBallPursuit cmdBallPursuit;

    // public StateTracker stateTracker = StateTracker.getInstance();

 
    // RobotTracker robotTracker = RobotTracker.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(6);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public double startTime = 0;

    public int reverse = -1;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight shooterLimelight, ballLimelight;
    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public FalconDrive drive = FalconDrive.getInstance();
    public Hopper hopper = Hopper.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Sidekick sidekick = Sidekick.getInstance();
    public Intake intake = Intake.getInstance();
    public StateTracker stateTracker = StateTracker.getInstance();

    private boolean teleopKinematics = false;

    static EKF ekf = new EKF(0, 0, Math.PI/2, 0, 0, 10, 10, 0.66,//0.9652,
    0.01, 1e-3, 0.01, 0.01);

    ArrayList<Double> KxList = new ArrayList<Double>();
    ArrayList<Double> KyList = new ArrayList<Double>();
    ArrayList<Double> KthetaList = new ArrayList<Double>();
    ArrayList<Double> KvlList = new ArrayList<Double>();
    ArrayList<Double> KvrList = new ArrayList<Double>();

    private double[] inputArray = new double[4];
    private double[] kinematicArray = new double[6];
    private double[] outputArray;
    private double currentTime, previousTime, printerTime, initTime;

    public static Pose2d ekfPosition;

    public CmdAlignShootTeleop alignCmd;
    public AutoSimple autoSimple;
    public AutoLessSimple autoLessSimple;

    public ErrorCatcherUtility errorCatcher;

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

        shooterLimelight = new Limelight("limelight-sog", -26.0, 0, 0, 30);
        ballLimelight = new Limelight("limelight-pog", Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_HEIGHT,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        drive.resetGyro();

        hopper.register();

        shooter.enable();
        shooter.setSetpointAndTolerance(0);
        sidekick.enable();
        sidekick.setState(Sidekick.ShooterState.DEFAULT);

        shooter.setState(Shooter.ShooterState.MID_RANGE);
        alignCmd = new CmdAlignShootTeleop(shooterLimelight, driveCmdRunning, 0, 26);

        

        //errorCatcher = new ErrorCatcherUtility(CanChain, limelights, drive);
        /*
        NarwhalDashboard.addButton("ErrorCatcher", (boolean down) -> {
            if(down) {
                errorCatcher.testEverything();
            }
        });

        NarwhalDashboard.addButton("VelocityTester", (boolean down) -> {
            if(down) {
                errorCatcher.velocityTester();
            }
        });
        */

        setupLimelights(ballLimelight,shooterLimelight);
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "Intake");

        listenerRight.nameControl(new Button(10), "MoveArmDown");
        listenerRight.nameControl(new Button(8), "MoveArmUp");

        listenerRight.nameControl(new Button(2), "Shoot");
        listenerLeft.nameControl(new Button(2), "ShootNotAligned");
        
        // listenerRight.nameControl(new Button(4), "Auto Intake");

        listenerRight.nameControl(new Button(3), "EmptyHopper");

        listenerRight.nameControl(new Button(7), "SetOverYonder");
        listenerRight.nameControl(new Button(9), "SetMiddling");
        listenerRight.nameControl(new Button(11), "SetIntimate");

        listenerRight.nameControl(new Button(12), "ResetBallCount");

        

        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "REVERSE");

        // listenerLeft.nameControl(new Button(12), "SetGreen");
        // listenerLeft.nameControl(new Button(11), "SetYellow");
        // listenerLeft.nameControl(new Button(9), "SetBlue");
        // listenerLeft.nameControl(new Button(7), "SetRed");

        listenerLeft.nameControl(new Button(11), "Increment Ball Count");
        listenerLeft.nameControl(new Button(12), "Decrement Ball Count");

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
        });

        listenerRight.addButtonUpListener("Intake", () -> {
            hopper.stopIntake();
        });

        listenerRight.addButtonDownListener("Shoot", () -> {
            stateTracker.enterShoot();
            scheduler.schedule(alignCmd);

        });

        listenerRight.addButtonUpListener("Shoot", () -> {
            stateTracker.exitShoot();
            alignCmd.cancel();
            //shooter.setSetpoint(0);
            driveCmdRunning.isRunning = true;

        });

        listenerLeft.addButtonDownListener("ShootNotAligned", () -> {
            shooter.isAligned = true; // kind of weird but need it to shoot 
            hopper.runIntake();
            sidekick.shoot();
            shooter.shoot();
        });

        listenerLeft.addButtonUpListener("ShootNotAligned", () -> {
            hopper.stopIntake();
            sidekick.counterShoot();
            shooter.counterShoot();
            hopper.unshoot = true;
            driveCmdRunning.isRunning = true;
            shooter.isAligned = false; // make sure we stop lying to the robot
        });

        // listenerRight.addButtonDownListener("Auto Intake", () -> {
        //     hopper.runIntake();
        //     scheduler.schedule(cmdBallPursuit);
        // });

        // listenerRight.addButtonUpListener("Auto Intake", () -> {
        //     hopper.stopIntake();
        //     cmdBallPursuit.cancel();
        //     driveCmdRunning.isRunning = true;
        // });

        listenerRight.addButtonDownListener("EmptyHopper", () -> {
            hopper.runHopperOpp();
            hopper.runIntakeOpp();
        });

        listenerRight.addButtonUpListener("EmptyHopper", () -> {
            hopper.stopHopper();
            hopper.stopIntake();
        });

        listenerRight.addButtonDownListener("MoveArmDown", () -> {
            hopper.moveArmDown();
        });

        listenerRight.addButtonUpListener("MoveArmDown", () -> {
            intake.stopArm();
        });

        listenerRight.addButtonDownListener("MoveArmUp", () -> {
            hopper.moveArmUp();
        });

        listenerRight.addButtonUpListener("MoveArmUp", () -> {
            intake.stopArm();
        });

        listenerRight.addButtonDownListener("ResetBallCount", () -> {
            hopper.resetBallCount();
        });

        listenerLeft.addButtonDownListener("REVERSE", () -> {
            reverse *= -1;
        });

        /*
        listenerLeft.addButtonDownListener("REVERSEHOPPER", () -> {
           hopper.reverseIntake();
        });

        listenerLeft.addButtonUpListener("REVERSEHOPPER", () -> {
            hopper.stopHopper();
         });
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

        // listenerLeft.addButtonDownListener("SetGreen", () -> {
        //     shooter.setState(Shooter.ShooterState.GREEN);
        // });
        // listenerLeft.addButtonDownListener("SetYellow", () -> {
        //     shooter.setState(Shooter.ShooterState.YELLOW);
        //     //shooter.setSetpoint(Shooter.ShooterState.YELLOW.shooterRPM);
        // });
        // listenerLeft.addButtonDownListener("SetBlue", () -> {
        //     shooter.setState(Shooter.ShooterState.BLUE);
        // });
        // listenerLeft.addButtonDownListener("SetRed", () -> {
        //     shooter.setState(Shooter.ShooterState.RED);
        // });

        listenerLeft.addButtonDownListener("Increment Ball Count", () -> {
            hopper.ballCount++;
        });

        listenerLeft.addButtonDownListener("Decrement Ball Count", () -> {
            hopper.ballCount--;
        });

    }

    @Override
    protected void constructAutoPrograms() {
        cmdBallPursuit = new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,42 * Angle.DEGREES);
        //scheduler.schedule(cmdBallPursuit);
        
        // cmdBallIntake = new CmdBallIntake(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        autoSimple = new AutoSimple(shooterLimelight, driveCmdRunning, 0, new PathFinding(), drive, hopper);
        autoLessSimple = new AutoLessSimple(shooterLimelight, driveCmdRunning, 0, new PathFinding(), drive, hopper);

        NarwhalDashboard.addAuto("Find ball maybe", cmdBallIntake);

        // cmdBallPursuit = new CmdBallPursuit(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        //NarwhalDashboard.addAuto("pog", cmdBallPursuit);
    }

    @Override
    protected void teleopPeriodic() {
        if (teleopKinematics){
            currentTime=RobotController.getFPGATime()/1000000.0;
            //currentTime = currentTime*1e-06;
            //I'm not sure how to check if new readings are available so right now we are running predict and update every time
            inputArray[0] = drive.getAngle() * Math.PI / 180.0;
            inputArray[1] = drive.getLeftSpeed() * 0.0254;
            inputArray[2] = drive.getRightSpeed() * 0.0254;
            inputArray[3] = currentTime-previousTime;
            // where EKF is run
            outputArray = ekf.runFilter(inputArray);

            
            kinematicArray[0] = KxList.get(KxList.size()-1);
            kinematicArray[1] = KyList.get(KyList.size()-1);
            kinematicArray[2] = drive.getAngle() * Math.PI / 180.0;
            kinematicArray[3] = drive.getLeftSpeed() * 0.0254;
            kinematicArray[4] = drive.getRightSpeed() * 0.0254;
            kinematicArray[5] = currentTime-previousTime;

            outputArray = ekf.testFunction(kinematicArray);

            KxList.add(outputArray[0]);
            KyList.add(outputArray[1]);
            KthetaList.add(outputArray[2]);
            KvlList.add(outputArray[3]);
            KvrList.add(outputArray[4]);
        
            //Log.info("EKF", "X: " + outputArray[0] + " Y: " + outputArray[1] + " THETA" + outputArray[2]);
            ekfPosition=new Pose2d(outputArray[0], outputArray[1], new Rotation2d(outputArray[2]));
            Log.info("EKF", ekfPosition.toString());




            previousTime=currentTime;
            //Log.info("MainAthos", ((Double) robotTracker.getOdometry().getTranslation().getX()).toString());

        }

    }

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

        SmartDashboard.putNumber("Shooter RPM", shooter.getMeasurement());
        SmartDashboard.putNumber("Sidekick RPM", sidekick.getMeasurement());
        SmartDashboard.putString("Shooter isReady", String.valueOf(shooter.isReady()));
        SmartDashboard.putString("Sidekick isReady", String.valueOf(sidekick.isReady()));

        SmartDashboard.putNumber("Hopper Ball Count", hopper.ballCount);
        SmartDashboard.putString("Hopper State", hopper.getState().toString());
        SmartDashboard.putBoolean("Shooter isAligned", shooter.isAligned);


    }

    @Override
    protected void teleopInit() {
        hopper.stopHopper();
        shooterLimelight.setLEDMode(LEDMode.OFF);
        Log.info("MainGrogu", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;

        if (teleopKinematics){
            drive.resetGyro();
            //scheduler.resume();

            KxList.add((double) 0);
            KyList.add((double) 0);
            KthetaList.add(Math.PI/2);
            KvlList.add((double) 0);
            KvrList.add((double) 0);

            initTime=RobotController.getFPGATime()/1000000.0;
        }



    }

    @Override
    protected void autonomousInit() {
        Log.info("MainGrogu", "moving arm down");
        // hopper.moveArmDown();
        hopper.moveArmUpAuto();

       // hopper.stopHopper();
        drive.resetGyro();
        
        // cmdBallIntake = new CmdBallIntake(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        // trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";
        // Log.info("MainAthos", "going into autonomousinit");
        // //scheduler.resume();
        // //drive.setAutoTrajectory(trajectory, false);
        // //drive.startTrajectory();
        // //scheduler.resume();

        KxList.add((double) 0);
        KyList.add((double) 0);
        KthetaList.add(Math.PI/2);
        KvlList.add((double) 0);
        KvrList.add((double) 0);


        // //use this for galactic search
        // hopper.runIntake();
        // PathFinding pathfinder = new PathFinding();
        // new PathRunner(pathfinder, drive).schedule();
        
        // startTime = Timer.getFPGATimestamp();

        // initTime=RobotController.getFPGATime()/1000000.0;


        // cmdBallPursuit = new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,42 * Angle.DEGREES);
        // scheduler.schedule(cmdBallIntake);

        scheduler.schedule(autoLessSimple);
        //scheduler.schedule(autoLessSimple);
    }


    // Prints EKF pos in autonomous
    @Override
    protected void autonomousPeriodic() {
        //hopper.resetBallCount();
        currentTime=RobotController.getFPGATime()/1000000.0;
        //currentTime = currentTime*1e-06;
        //I'm not sure how to check if new readings are available so right now we are running predict and update every time
        inputArray[0] = drive.getAngle() * Math.PI / 180.0;
        inputArray[1] = drive.getLeftSpeed() * 0.0254;
        inputArray[2] = drive.getRightSpeed() * 0.0254;
        inputArray[3] = currentTime-previousTime;
       // where EKF is run

        kinematicArray[0] = KxList.get(KxList.size()-1);
        kinematicArray[1] = KyList.get(KyList.size()-1);
        kinematicArray[2] = drive.getAngle() * Math.PI / 180.0;
        kinematicArray[3] = drive.getLeftSpeed() * 0.0254;
        kinematicArray[4] = drive.getRightSpeed() * 0.0254;
        kinematicArray[5] = currentTime-previousTime;

        outputArray = ekf.testFunction(kinematicArray);

        KxList.add(outputArray[0]);
        KyList.add(outputArray[1]);
        KthetaList.add(outputArray[2]);
        KvlList.add(outputArray[3]);
        KvrList.add(outputArray[4]);
       
        //Log.info("EKF", "X: " + outputArray[0] + " Y: " + outputArray[1] + " THETA" + outputArray[2]);
        ekfPosition=new Pose2d(outputArray[0], outputArray[1], new Rotation2d(outputArray[2]));
        // Log.info("EKF", ekfPosition.toString());

        previousTime=currentTime;
    }


    @Override
    protected void disabledInit() {
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