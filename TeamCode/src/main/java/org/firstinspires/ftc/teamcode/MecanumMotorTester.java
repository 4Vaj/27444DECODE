package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.webkit.WebStorage;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Mecanum Motor Tester - 27444 DECODE", group = "Aveion")
//@Disabled
public class MecanumMotorTester extends OpMode{

    //IMU
    BHI260IMU imu;
    Orientation angles;
    //Timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();

    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime timeToComplete = new ElapsedTime();

    // MOTOR ENCODER VALUES
    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_MM = 104;
    static final double GEAR_RATIO = 1.0;

    //Front to back wheel centers measurement: 300.839mm
    //Left to right wheel centers measurement: 348.160mm
    static final double ROBOT_WIDTH_MM = 300.839;
    static final double ROBOT_LENGTH_MM = 348.160;
    double wheelCircumference = Math.PI * WHEEL_DIAMETER_MM;




    //Independent Variables

    final double pushPos = 0.65;

    final double pushPosSet = 0.38;
    final double liftTime = 1;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Target Velocities
    final double VelocityZero = 1100;
    final double VelocityOne = 1200;
    final double VelocityTwo = 1500;
    final double VelocityThree = 2000;
    final double VelocityFour = 2500;

    //Dependent Variables

        //Mecanum Accuracy
        private int flTarget, frTarget, blTarget, brTarget;
        private double movePower, desiredHeading;
        private boolean mecanumActive;

        // Tuning Constants
        double kP_encoder = 0.0005;
        double kP_heading = 0.01;
        int encoderTolerance = 10;

    private double delay;
    private String sideString;
    private String colorString;

    private boolean shootRequested;

    private int step = 0;

    private boolean timerRunning;
    private double timerDuration = 0;
    private boolean intakeRunning;
    private double intakeDuration = 0;

    // Launcher Target and Minimumm Velocities
    private double LAUNCHER_TARGET_VELOCITY = 0;
    private double LAUNCHER_MIN_VELOCITY = 0;
    private double gear = 0;
    private String lifting = null;
    private boolean spinning;
    //Declare Motors
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx flywheel = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private DcMotor leftConveyor = null;
    private DcMotor rightConveyor = null;
    private Servo lift = null;

    private boolean countachFound;

    private double conveyPower = 0;


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
    }

    private enum MLiftState {
        IDLE,
        UP,
        DOWN,
    }

    // Starting color enum to determine what program to run
    private enum StartingColor{
        RED(1),
        BLUE(-1);

        final int sign;
        StartingColor(int sign){
            this.sign = sign;
        }
    }

    // Starting side enum to determine what program to run
    private enum StartingSide{
        GOAL(true),
        TRIANGLE(false);

        final boolean sign;
        StartingSide(boolean sign){
            this.sign = sign;
        }
    }

    //private LaunchState launchState;
    private LiftState LiftState;
    private MLiftState MLiftState;

    private StartingSide StartingSide;
    private StartingColor StartingColor;

    //Variables for each drive motor
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    /*/////////////////////////////////////////////////////////////////////////////////
    INITIALIZE
    /////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void init(){;

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();


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


        //launchState = LaunchState.IDLE;
        LiftState = LiftState.IDLE;
        MLiftState = MLiftState.IDLE;

        //Configuration
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftIntake = hardwareMap.get(CRServo.class, "leftFeed");
        rightIntake = hardwareMap.get(CRServo.class, "rightFeed");

        leftConveyor = hardwareMap.get(DcMotor.class, "leftConvey");
        rightConveyor = hardwareMap.get(DcMotor.class, "rightConvey");

        lift = hardwareMap.get(Servo.class, "lift");

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
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // Set Push Servo to down
        lift.setPosition(pushPosSet);

        // Default Target Velocity
        LAUNCHER_TARGET_VELOCITY = VelocityZero;

        // Checks
        StartingColor = StartingColor.RED;
        StartingSide = StartingSide.GOAL;
        intakeRunning = false;
        timerRunning = false;
        step = 0;
        delay = 0;
        mecanumActive = false;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    INITIALIZE LOOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        // Initialization Telemetry
        telemetry.addLine("Status : Initialized\n");
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        startMecanumMove(0, 305, 0, 0.5);

        timeToComplete.reset();
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    /*//////////////////////////////////////////////////////////////////////////////////////
    LOOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void loop() {


        // Get heading using IMU
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;

        // Telemetry

        // Powers
        telemetry.addData("FL Power", frontLeft.getPower());
        telemetry.addData("FR Power", frontRight.getPower());
        telemetry.addData("BL Power: ", backLeft.getPower());
        telemetry.addData("BR Power: ", backRight.getPower());

        // Positions
        telemetry.addData("\nFL Position", frontLeft.getCurrentPosition());
        telemetry.addData("FR Position", frontRight.getCurrentPosition());
        telemetry.addData("BL Position", backLeft.getCurrentPosition());
        telemetry.addData("BR Position", backRight.getCurrentPosition());

        // Velocities
        telemetry.addData("\nFL Velocity", frontLeft.getVelocity());
        telemetry.addData("FR Velocity", frontRight.getVelocity());
        telemetry.addData("BL Velocity", backLeft.getVelocity());
        telemetry.addData("BR Velocity", backRight.getVelocity());

        //Check if mecanum is active
        telemetry.addData("\nMecanum active", mecanumActive);

        telemetry.update();

        updateMecanum();
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    STOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void stop() {
        telemetry.addData("Time to complete :", timeToComplete.seconds());
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    FUNCTIONS
    /////////////////////////////////////////////////////////////////////////////////////*/
    int mmToTicks(double distanceMM){
        return (int)((distanceMM/wheelCircumference) * TICKS_PER_REV * GEAR_RATIO);
    }

    //Check Functions

    boolean mecanumMoveDone(){
        return !frontRight.isBusy() && !frontLeft.isBusy() && !backLeft.isBusy() && !backRight.isBusy();
    }
    void UpdateWait(){
        if(!timerRunning){
            return;
        }
        if(waitTimer.seconds() > timerDuration){
            timerRunning = false;
        }
    }
    void UpdateIntake(){
        if(!intakeRunning){
            return;
        }
        if(intakeTimer.seconds() < intakeDuration){
            leftIntake.setPower(conveyPower);
            rightIntake.setPower(conveyPower);
            leftConveyor.setPower(conveyPower);
            rightConveyor.setPower(conveyPower);
        }
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            leftConveyor.setPower(0);
            rightConveyor.setPower(0);
            intakeRunning = false;
        }
    }

    // Physical Functions
    void Wait(double time){
        waitTimer.reset();
        timerDuration = time;
        timerRunning = true;
    }
    void startMecanumMove(double xMM, double yMM, double rotationDeg, double power){
        //Convert rotation from degrees to approximate wheel linear distance (arc length)
        double rotationMM = Math.toRadians(rotationDeg) * ((ROBOT_LENGTH_MM + ROBOT_WIDTH_MM) / 2.0);

        // Calculate each wheel's linear distance (mm)
        double frontLeftMM  =  yMM  + xMM + rotationMM;
        double frontRightMM =  yMM  - xMM - rotationMM;
        double backLeftMM  =   yMM  - xMM + rotationMM;
        double backRightMM   = yMM  + xMM - rotationMM;

        // Convert to encoder ticks
        int flTicks = mmToTicks(frontLeftMM);
        int frTicks = mmToTicks(frontRightMM);
        int brTicks = mmToTicks(backRightMM);
        int blTicks = mmToTicks(backLeftMM);

        // Set move power and heading
        movePower = power;

        // Desired heading is the current heading when movement stops
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        desiredHeading = angles.firstAngle;

        // Activate the mecanum
        mecanumActive = true;

        // ruR using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void updateMecanum(){
        if (!mecanumActive) return;

        //Encoder errors
        int flError = flTarget - frontLeft.getCurrentPosition();
        int frError = frTarget - frontRight.getCurrentPosition();
        int blError = blTarget - backLeft.getCurrentPosition();
        int brError = brTarget - backRight.getCurrentPosition();

        // Check if all wheels reached target
        if (Math.abs(flError) < encoderTolerance &&
                Math.abs(frError) < encoderTolerance &&
                Math.abs(blError) < encoderTolerance &&
                Math.abs(brError) < encoderTolerance){
            // Stop motors
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            mecanumActive = false;
            return;
        }
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double currentHeading = angles.firstAngle;
        double headingError = desiredHeading - currentHeading;
        while (headingError > 180){
            headingError -= 360;
        }
        while (headingError < -180){
            headingError += 360;
        }
        double rotationCorrection = headingError * kP_heading;

        // Motor Powers
        double flPower = clamp(flError * kP_encoder + rotationCorrection, -movePower, movePower);
        double frPower = clamp(frError * kP_encoder + rotationCorrection, -movePower, movePower);
        double blPower = clamp(blError * kP_encoder + rotationCorrection, -movePower, movePower);
        double brPower = clamp(brError * kP_encoder + rotationCorrection, -movePower, movePower);

        //Apply Powers
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

    }
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }


    void MoveMecanum(double xMM, double yMM, double rotationDeg, double power){

        //Convert rotation from degrees to approximate wheel linear distance (arc length)
        double rotationMM = Math.toRadians(rotationDeg) * ((ROBOT_LENGTH_MM + ROBOT_WIDTH_MM) / 2.0);

        // Calculate each wheel's linear distance (mm)
        double frontLeftMM  =  yMM  + xMM + rotationMM;
        double frontRightMM =  yMM  - xMM - rotationMM;
        double backLeftMM  =   yMM  - xMM + rotationMM;
        double backRightMM   = yMM  + xMM - rotationMM;

        // Convert to encoder ticks
        int flTicks = mmToTicks(frontLeftMM);
        int frTicks = mmToTicks(frontRightMM);
        int brTicks = mmToTicks(backRightMM);
        int blTicks = mmToTicks(backLeftMM);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set target positions
        frontLeft.setTargetPosition(flTicks);
        frontRight.setTargetPosition(frTicks);
        backRight.setTargetPosition(brTicks);
        backLeft.setTargetPosition(blTicks);
        // Run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }
    void Intake(double time) {
        //Intake
        conveyPower = 1;
        intakeDuration = time;
        intakeTimer.reset();
        intakeRunning = true;
    }
    void Outtake(double time) {
        //Outtake
        conveyPower = -1;
        intakeDuration = time;
        intakeTimer.reset();
        intakeRunning = true;
    }

    void ManualLift(boolean liftRequested){
        switch (MLiftState){
            case IDLE:
                lifting = "";
                if(liftRequested){
                    MLiftState = MLiftState.UP;
                }
                break;
            case UP:
                lifting = "lifting";
                lift.setPosition(pushPos);
                liftTimer.reset();
                MLiftState = MLiftState.DOWN;
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
                    LiftState = LiftState.IDLE;
                }
                break;
        }
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
