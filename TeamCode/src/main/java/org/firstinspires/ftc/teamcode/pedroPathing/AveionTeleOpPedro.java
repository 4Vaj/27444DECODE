package org.firstinspires.ftc.teamcode.pedroPathing;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.clamp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;

@TeleOp(name="27444 DECODE TeleOp", group = "Aveion")
//@Disabled
public class AveionTeleOpPedro extends OpMode{
    //PEDRO PATHING
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

    //Timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime conveyTimer = new ElapsedTime();
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime jerkTimer = new ElapsedTime();

    //Independent Variables
    final double pushPos = 0.55;

    final double pushPosSet = 0.38;
    final double liftTime = 1.0;

    final double conveyTime = 1.0;
    final double conveyWaitTime = 0.25;
    final double jerkTime = 0.2;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double brakeTolerance = 150.0;

    // Drive Gears
    final double gearOne = 0.4;
    final double gearTwo = 0.6;
    final double gearThree = 1.0;

    // Target Velocities
    final double tolerance = 50.0;
    final double lineUpVelocity = 1100.0;
    final double middleVelocity = 1280.0;
    final double vertexVelocity = 1500.0;
    final double farVelocity = 1700.0;

    // Target Radii
    final double mediumRadius = 570.0; // 56.63
    final double farRadius = 90.0; // 90.55

    // Goal Coordinates
    final double cx = 134.0;
    final double cy = 138.0;

    // Lists For Interpolating Shooting Presets
    static final double[] D = {18, Math.hypot(26, 38), Math.hypot(62, 66)};
    static final double[] HOOD = {0.0, 1.0, 1.0};
    static final double[] VEL = {1100, 1280, 1500};





    // Declare Dependent Variables
    private double LAUNCHER_TARGET_VELOCITY = 0;
    private double LAUNCHER_MIN_VELOCITY = 0;
    private double gear = 0;
    private String lifting = null;
    private boolean spinning;
    private boolean conveying;
    private boolean braking;
    private double angleToGoalInDeg;

    //Declare Motors
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx flywheel = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private DcMotor leftConveyor = null;
    private DcMotor rightConveyor = null;
    private Servo lift = null;
    private Servo hood = null;
    private double robotX = 0;
    private double robotY = 0;

    private double robotHeading = 0;
    private double kHeading = 0;
    private boolean aimming = false;



    private boolean countachFound;


    private enum LaunchState {
        IDLE,
        SPOOL,
        PUSH,
        LAUNCH,
        LAUNCHING,
    }
    private enum LiftState {
        IDLE,
        SPOOL,
        LIFT,
        LIFTING,
        CONVEY,
    }

    private enum MLiftState {
        IDLE,
        UP,
        DOWN,
    }
    // Launch Mode
    private enum LaunchMode {
        LineUp,
        Medium,
        Vertex,
        Far
    }
    private LaunchMode launchMode;
    //private LaunchState launchState;
    private LiftState LiftState;
    private MLiftState MLiftState;

    //Variables for each drive motor
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    /*/////////////////////////////////////////////////////////////////////////////////
    INITIALIZE
    /////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void init(){
        // Determine Resource IDs for sounds built into the RC application.
        int countachSoundID = hardwareMap.appContext.getResources().getIdentifier("countachlouder", "raw", hardwareMap.appContext.getPackageName());
        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (countachSoundID != 0) {
            countachFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, countachSoundID);
        }

        telemetry.addData("Countach resource",   countachFound ?   "Found" : "NOT found\n Add countach.wav to /src/main/res/raw" );

        if (countachFound) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, countachSoundID);
            telemetry.addLine("Playing Sound.\n");
            telemetry.update();
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();



        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();



        //launchState = LaunchState.IDLE;
        LiftState = LiftState.IDLE;
        MLiftState = MLiftState.IDLE;

        //Configuration
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftIntake = hardwareMap.get(CRServo.class, "leftFeed");
        rightIntake = hardwareMap.get(CRServo.class, "rightFeed");

        leftConveyor = hardwareMap.get(DcMotor.class, "leftConvey");
        rightConveyor = hardwareMap.get(DcMotor.class, "rightConvey");

        lift = hardwareMap.get(Servo.class, "lift");
        hood = hardwareMap.get(Servo.class, "hood");


        //Motor Directions

        //Drive Motors
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Launch Motors
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        //Conveyor Motors
        leftConveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightConveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo Directions

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);


        //Motor Behaviors

        //Launch Motors
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Drive Motors
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);

        //Conveyor Motors
        leftConveyor.setZeroPowerBehavior(BRAKE);
        rightConveyor.setZeroPowerBehavior(BRAKE);

        //Servo Behaviors

        //Feeders
        leftIntake.setPower(STOP_SPEED);
        rightIntake.setPower(STOP_SPEED);

        //PIDF Coefficients (i actually don't know how these work)
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        //Checks
        // Set Push Servo to down
        lift.setPosition(pushPosSet);
        LAUNCHER_TARGET_VELOCITY = lineUpVelocity;
        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - tolerance;
        spinning = false;
        conveying = false;
        lifting = "";

        // Initially start with gear three
        gear = gearThree;
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    INITIALIZE LOOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        // Drive Gears
        if (gamepad1.dpad_right){
            gear = gearThree;
        }
        else if (gamepad1.dpad_up){
            gear = gearTwo;
        }
        else if (gamepad1.dpad_left){
            gear = gearOne;
        }
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized\n\n\n");
        telemetry.addData("Drive Power", gear);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addLine("Miscellaneous:");
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.update();
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        follower.startTeleopDrive();
        automatedDrive = true;
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {
        // Checks
        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - tolerance;
        Brake();
        follower.update();
        robotX = follower.getPose().getX();
        robotY = follower.getPose().getY();
        robotHeading = follower.getHeading();
        angleToGoalInDeg = Math.atan2(cx - robotX, cy - robotY);
        if(gamepad1.leftStickButtonWasPressed())
        {
            kHeading += 0.01;
        }
        if(gamepad1.rightStickButtonWasPressed())
        {
            kHeading -= 0.01;
        }


        // Mecanum Drive
        if (!automatedDrive) {
            //Normal Mecanum Controls
            if(!aimming){
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * gear,
                    -gamepad1.left_stick_x * gear,
                    -gamepad1.right_stick_x * gear,
                    true // Robot Centric
                );
            }
            //Aimming Controls
            else if (aimming) {
                // --- AUTO AIM ---
                double angleToGoal = Math.atan2(cy - robotY, cx - robotX);
                double headingError = angleWrap(angleToGoal - robotHeading);


                double turn = kHeading * headingError;
                turn = clamp(turn, -1.0, 1.0);


                // --- TELEOP DRIVE WITH AUTO TURN ---
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * gear,   // from left stick
                        -gamepad1.left_stick_x * gear,    // from left stick
                        turn,           // AUTO AIM replaces right stick
                        false            // field-centric
                );
                updateConfig(distanceToGoal(robotX, robotY));
            }
        }

        // Hold left bumper to aim
        if (gamepad1.left_bumper){
            aimming = true;
        }
        else if (!gamepad1.left_bumper){
            aimming = false;
        }

        // Launch Settings Buttons
        if (gamepad2.dpad_left){ // Line Up
            LAUNCHER_TARGET_VELOCITY = lineUpVelocity;
            hood.setPosition(0.0);
        }
        else if (gamepad2.dpad_up){ // Between goal and vertex
            LAUNCHER_TARGET_VELOCITY = middleVelocity;
            hood.setPosition(1.0);
        }
        else if (gamepad2.dpad_right){ // Vertex of large triangle
            LAUNCHER_TARGET_VELOCITY = vertexVelocity;
            hood.setPosition(1.0);
        }
        else if (gamepad2.dpad_down){ // Far
            LAUNCHER_TARGET_VELOCITY = farVelocity;
            hood.setPosition(1.0);
        }

        // Manual Launch Velocities
        else if (gamepad2.rightBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY += 10;
        }
        else if (gamepad2.leftBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY -= 10;
        }

        // Launch mode settings

        /*if(launchMode == LaunchMode.LineUp){
            LAUNCHER_TARGET_VELOCITY = lineUpVelocity;
            hood.setPosition(0.0);
        } else if (launchMode == LaunchMode.Medium) {
            LAUNCHER_TARGET_VELOCITY = middleVelocity;
            hood.setPosition(1);
        } else if (launchMode == LaunchMode.Vertex){
            LAUNCHER_TARGET_VELOCITY = vertexVelocity;
            hood.setPosition(1);
        } else if (launchMode == LaunchMode.Far) {
            LAUNCHER_TARGET_VELOCITY = farVelocity;
            hood.setPosition(1);
        }*/

        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 200;


        //Manual Controls
        if (gamepad2.cross) {
            spinning = true;
            flywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }

        else if (gamepad2.circle) {
            spinning = false;
            if (braking) {
                startBrake();
            }
        }
        // Drive Gears
        if (gamepad1.dpad_right){
            gear = gearThree;
        }
        else if (gamepad1.dpad_up){
            gear = gearTwo;
        }
        else if (gamepad1.dpad_left){
            gear = gearOne;
        }

        // Conveying
        leftIntake.setPower(-gamepad2.left_trigger);
        rightIntake.setPower(-gamepad2.left_trigger);
        leftConveyor.setPower(-gamepad2.left_trigger);
        rightConveyor.setPower(-gamepad2.left_trigger);

        leftIntake.setPower(gamepad2.right_trigger);
        rightIntake.setPower(gamepad2.right_trigger);
        leftConveyor.setPower(gamepad2.right_trigger);
        rightConveyor.setPower(gamepad2.right_trigger);

        // Method controls
        Shoot(gamepad2.triangle); // Auto Shoot (slower cycling)
        ManualLift(gamepad2.squareWasPressed());

        telemetry.addData("Spinning", spinning);
        //telemetry.addData("Lift", lifting);
        //telemetry.addData("Conveying", conveying);
        telemetry.addData("Flywheel RPM", flywheel.getVelocity());
        //telemetry.addData("Lift", lift.getPosition());
        telemetry.addData("Velocity Within Tolerance", (flywheel.getVelocity() > LAUNCHER_MIN_VELOCITY));
        //telemetry.addLine("12345678912345678912345678912345678912");
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("D-pad left", "Line Up");
        telemetry.addData("D-pad up", "Middle");
        telemetry.addData("D-pad right", "Edge");
        telemetry.addData("D-pad down", "Small Triangle");
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("Drive Power", gear);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Robot Position", follower.getPose());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("kHeading", kHeading);
        telemetry.update();
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    STOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void stop() {
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    METHODS
    /////////////////////////////////////////////////////////////////////////////////////*/

    void mecanumDrive(float forward, float strafe, float rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        float denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        frontLeftPower = (( forward  + strafe + rotate) * gear) / denominator;
        frontRightPower = ((forward  - strafe - rotate) * gear) / denominator;
        backLeftPower = ((  forward  - strafe + rotate) * gear ) / denominator;
        backRightPower = (( forward  + strafe - rotate) * gear ) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }



//

    void ManualLift(boolean liftRequested){
        switch (MLiftState){
            case IDLE:
                lifting = "";
                if(liftRequested){
                    lifting = "Waiting to lift";
                    MLiftState = MLiftState.UP;
                }
                break;
            case UP:
                if (flywheel.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    lifting = "Lifting";
                    lift.setPosition(pushPos);
                    liftTimer.reset();
                    MLiftState = MLiftState.DOWN;
                }
                break;
            case DOWN:
                if (liftTimer.seconds() > liftTime) {
                    lifting = "";
                    lift.setPosition(pushPosSet);
                    MLiftState = MLiftState.IDLE;
                }
                break;
        }
    }
    void Shoot(boolean shotRequested) {
        switch (LiftState){
            case IDLE:
                spinning = false;
                if (shotRequested){
                    LiftState = LiftState.SPOOL;
                }
                break;
            case SPOOL:
                spinning = true;
                flywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVelocity() > LAUNCHER_MIN_VELOCITY)
                {
                    LiftState = LiftState.LIFT;
                }
                break;
            case LIFT:
                lifting = "Lifting";
                lift.setPosition(pushPos);
                liftTimer.reset();
                LiftState = LiftState.LIFTING;
                break;
            case LIFTING:
                if (liftTimer.seconds() > liftTime) {
                    lifting = "";
                    lift.setPosition(pushPosSet);
                    waitTimer.reset();
                    conveyTimer.reset();
                    LiftState = LiftState.CONVEY;
                }
                break;
            case CONVEY:
                if (waitTimer.seconds() > conveyWaitTime){
                    conveying = true;
                    leftConveyor.setPower(1);
                    rightConveyor.setPower(1);
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                    if (conveyTimer.seconds() > conveyTime){
                        conveying = false;
                        LiftState = LiftState.IDLE;
                    }
                }
                break;
        }
    }
    public void startBrake(){
        braking = true;
        jerkTimer.reset();
    }
    public void Brake(){
        if (!braking) return;
        double velocity = flywheel.getVelocity();
        if (Math.abs(velocity) < brakeTolerance){
            braking = false;
            return;
        }
        if (jerkTimer.seconds() < jerkTime) {
            flywheel.setVelocity(-Math.copySign(2000,velocity));
        } else {
            flywheel.setVelocity(0);
        }
    }

    public double distanceToGoal(double rX, double rY){
        return Math.hypot(cx - rX, cy - rY);
    }
    static double lerp(double start, double end, double iParam) {
        return start + (end - start) * iParam; // Linear Interpolation Equation
    }

    static double lookupLerp(double dist, double[] D, double[] Y){ // Y is essentially the value we need to find so RPM and Hood Position
        if (dist <= D[0]) return Y[0]; // Clamp near
        if (dist >= D[D.length - 1]) return Y[Y.length - 1]; // Clamp far

        for (int i = 0; i < D.length - 1; i++) { // for loop that runs for every distance
            if (dist >= D[i] && dist <= D[i + 1]) { // if distance is between two points
                double t = (dist - D[i]) / (D[i + 1] - D[i]); // calculate t value by dividing distance past a point by distance between points
                return lerp(Y[i], Y[i + 1], t); // returns lerped value
            }
        }
        return Y[Y.length - 1];
    }
    public void updateConfig(double distInches){
        double hoodTarget = lookupLerp(distInches, D, HOOD);
        double rpmTarget = lookupLerp(distInches, D, VEL);

        hood.setPosition(hoodTarget);
        flywheel.setVelocity(rpmTarget);
    }
    double angleWrap(double a){ // Calculates how much the robot needs to turn
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }


    /*void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPOOL;
                }
                break;
            case SPOOL:
                flywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (flywheel.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                     launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftConveyor.setPower(FULL_SPEED);
                rightConveyor.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftConveyor.setPower(STOP_SPEED);
                    rightConveyor.setPower(STOP_SPEED);
                }
                break;
        }
    }*/
}
