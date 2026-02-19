package org.firstinspires.ftc.teamcode.pedroPathing;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@Autonomous(name = "Aveion Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AveionPedroAuto2 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    //Timers
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime conveyTimer = new ElapsedTime();
    ElapsedTime conveyWait = new ElapsedTime();


    // Motors and Servos
    private DcMotorEx flywheel = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private DcMotor leftConveyor = null;
    private DcMotor rightConveyor = null;
    private Servo lift = null;
    private Servo hood = null;

    private boolean countachFound;

    // Variables
    final double liftUp = 0.55;

    final double liftDown = 0.38;
    final double liftTime = 1;
    final double conveyTime = 0.75;
    final double conveyWaitTime = 1;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0;
    final double FULL_SPEED = 1;

    // Target Velocities
    final double tolerance = 50;
    final double lineUpVelocity = 1100;
    final double middleVelocity = 1280;
    final double vertexVelocity = 1500;
    final double farVelocity = 1700;

    // Declare Dependent Variables
    private double LAUNCHER_TARGET_VELOCITY = 0;
    private double LAUNCHER_MIN_VELOCITY = 0;
    private boolean lifting;
    private boolean spinning;
    private boolean conveying;
    private int shots;
    private int requestedShots;

    // State Machines
    private enum LiftState {
        IDLE,
        SPOOL,
        LIFT,
        LIFTING,
        CONVEY,
        STOP_CONVEY,
    }
    private LiftState liftState;

    /*/////////////////////////////////////////////////////////////////////////////////
    INITIALIZE
    /////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void init() {
        // Startup Sound
        int countachSoundID = hardwareMap.appContext.getResources().getIdentifier("countachlouder", "raw", hardwareMap.appContext.getPackageName());
        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (countachSoundID != 0) {
            countachFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, countachSoundID);
        }

        if (countachFound) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, countachSoundID);
        }

        //Pedro
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87, 9, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        // Set State Machines
        liftState = liftState.IDLE;

        //Initialize Motors and Servos
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftIntake = hardwareMap.get(CRServo.class, "leftFeed");
        rightIntake = hardwareMap.get(CRServo.class, "rightFeed");

        leftConveyor = hardwareMap.get(DcMotor.class, "leftConvey");
        rightConveyor = hardwareMap.get(DcMotor.class, "rightConvey");

        lift = hardwareMap.get(Servo.class, "lift");
        hood = hardwareMap.get(Servo.class, "hood");

        //Directions
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        leftConveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightConveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Behaviors
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftConveyor.setZeroPowerBehavior(BRAKE);
        rightConveyor.setZeroPowerBehavior(BRAKE);

        leftIntake.setPower(STOP_SPEED);
        rightIntake.setPower(STOP_SPEED);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Checks
        lift.setPosition(liftDown); // Servo Down
        LAUNCHER_TARGET_VELOCITY = farVelocity;
        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - tolerance;
        spinning = false;
        conveying = false;
        lifting = false;
        shots = 0;
        hood.setPosition(1);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        hood.setPosition(1);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain DriveToShoot;
        public PathChain LineUpSpike1;
        public PathChain GrabArtifacts;
        public PathChain BackUpFromSpike;
        public PathChain DrivetoShoot2;

        public Paths(Follower follower) {
            DriveToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(87.000, 9.000),
                                    new Pose(83.000, 73.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            LineUpSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.000, 73.200),
                                    new Pose(83.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            GrabArtifacts = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.000, 27.000),
                                    new Pose(125.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            BackUpFromSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.000, 27.000),
                                    new Pose(83.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            DrivetoShoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.000, 27.000),
                                    new Pose(83.000, 73.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (pathState){
            case 0:
                setPathState(1);
                break;
            case 1:
                follower.followPath(paths.DriveToShoot, true);
                setPathState(2);
                break;
            case 2:
                if(!follower.isBusy()) {
                    Shoot(true);
                    requestShots(3);
                    if(shots == requestedShots){
                        setPathState(3);
                        flywheel.setVelocity(0);
                        waitTimer.reset();
                    }
                }
                break;
            case 3:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    follower.followPath(paths.LineUpSpike1, true);
                    setPathState(4);
                    waitTimer.reset();
                }
                break;
            case 4:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    follower.followPath(paths.GrabArtifacts, .4, true);
                    setPathState(5);
                    Convey(true);
                    waitTimer.reset();
                }
                break;
            case 5:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    follower.followPath(paths.BackUpFromSpike, true);
                    setPathState(6);
                    waitTimer.reset();
                }
                break;
            case 6:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    follower.followPath(paths.DrivetoShoot2, true);
                    setPathState(7);
                    waitTimer.reset();
                }
                break;
            case 7:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    follower.followPath(paths.LineUpSpike1, true);
                    setPathState(8);
                    waitTimer.reset();
                }
                break;
            case 8:
                if(!follower.isBusy() && waitTimer.milliseconds() > 2500){
                    setPathState(-1);
                }
            break;
        }
        return pathState;
    }
    public void setPathState(int pState){
        pathState = pState;
    }

    void Shoot(boolean shotRequested){
        switch(liftState){
            case IDLE:
                spinning = false;
                if(shotRequested && shots < requestedShots){
                    liftState = liftState.SPOOL;
                }
                break;
            case SPOOL:
                spinning = true;
                flywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    liftState = liftState.LIFT;
                }
                break;
            case LIFT:
                lifting = true;
                lift.setPosition(liftUp);
                liftTimer.reset();
                liftState = liftState.LIFTING;
                break;
            case LIFTING:
                if (liftTimer.seconds() > liftTime){
                    lifting = false;
                    lift.setPosition(liftDown);
                    conveyWait.reset();
                    liftState = liftState.CONVEY;
                }
                break;
            case CONVEY:
                if (conveyWait.seconds() > conveyWaitTime){
                    conveying = true;
                    leftConveyor.setPower(1);
                    rightConveyor.setPower(1);
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                    conveyTimer.reset();
                    liftState = liftState.STOP_CONVEY;
                }
                break;
            case STOP_CONVEY:
                if(conveyTimer.seconds() > conveyTime){
                    conveying = false;
                    leftConveyor.setPower(0);
                    rightConveyor.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    shots++;
                    liftState = liftState.IDLE;
                }
                break;
        }
    }
    public void requestShots(int shots){
        requestedShots = shots;
    }

    public void Convey(boolean conveyRequest){
        conveying = conveyRequest;
        if (conveyRequest){
            leftConveyor.setPower(1);
            rightConveyor.setPower(1);
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        else {
            leftConveyor.setPower(0);
            rightConveyor.setPower(0);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
}