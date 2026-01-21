package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.media.tv.TvContract;
import android.webkit.WebStorage;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.IMU;
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
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="27444 DECODE Auto", group = "Aveion")
//@Disabled
public class AveionAutonomous extends OpMode{

    // IMU
    IMU imu;
    RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.UP);

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
    final double liftTime = 0.25;
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
        GOAL,
        TRIANGLE,
        TTG; //Triangle To Goal
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
    public void init(){

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(hubOrientation));

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
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    INITIALIZE LOOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        // Pick a velocity to shoot at
        if (gamepad1.dpadLeftWasPressed()){
            LAUNCHER_TARGET_VELOCITY = VelocityOne;
        }
        else if (gamepad1.dpadUpWasPressed()){
            LAUNCHER_TARGET_VELOCITY = VelocityTwo;
        }
        else if (gamepad1.dpadRightWasPressed()){
            LAUNCHER_TARGET_VELOCITY = VelocityThree;
        }
        else if (gamepad1.dpadDownWasPressed()){
            LAUNCHER_TARGET_VELOCITY = VelocityFour;
        }
        else if (gamepad1.psWasPressed()){
            LAUNCHER_TARGET_VELOCITY = VelocityZero;
        }
        else if (gamepad1.leftBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY -= 50;
        }
        else if (gamepad1.rightBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY += 50;
        }
        // Create a configuration
        else if (gamepad1.circleWasPressed()){
            StartingColor = StartingColor.RED;
        }
        else if (gamepad1.crossWasPressed()){
            StartingColor = StartingColor.BLUE;
        }
        else if (gamepad1.triangleWasPressed()){
            StartingSide = StartingSide.TRIANGLE;
        }
        else if (gamepad1.squareWasPressed()){
            StartingSide = StartingSide.GOAL;
        }
        else if (gamepad1.square && gamepad1.triangle){
            StartingSide = StartingSide.TTG;
        }
        // Set a delay for alliance
        else if (gamepad1.optionsWasPressed()){
            delay++;
        }
        else if (gamepad1.shareWasPressed()){
            delay--;
        }

        if (StartingColor == StartingColor.RED){
            colorString = "Red";
        }
        else{
            colorString = "Blue";
        }

        if (StartingSide == StartingSide.GOAL){
            sideString = "Goal";
        }
        else if (StartingSide == StartingSide.TRIANGLE){
            sideString = "Triangle";
        }
        else {
            sideString = "Triangle To Goal";
        }


        // Initialization Telemetry
        telemetry.addLine("Status : Initialized\n");
        telemetry.addLine("Please create a configuration to run auto (default: Red, Goal)");
        telemetry.addLine("Circle/B: Red");
        telemetry.addLine("Cross/A: Blue");
        telemetry.addLine("Square/X: Goal");
        telemetry.addLine("Triangle/Y: Triangle");
        telemetry.addLine("Triangle + Square/Y + X: Triangle To Goal");
        telemetry.addData("Config", colorString + ", " + sideString);
//        telemetry.addData("Color:", colorString);
//        telemetry.addData("Side:", sideString);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addLine("Please choose a velocity to run auto (default: "+VelocityZero+" ).");
        telemetry.addData("D-pad left", VelocityOne);
        telemetry.addData("D-pad up", VelocityTwo);
        telemetry.addData("D-pad down", VelocityThree);
        telemetry.addData("D-pad right", VelocityFour);
        telemetry.addData("PS/Home :", VelocityZero);
        telemetry.addLine("Right Bumper : add 50");
        telemetry.addLine("Left Bumper : subtract 50");
        telemetry.addData("Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addLine("Delay for alliance? (Default 0).");
        telemetry.addLine("Option : Add 1 second");
        telemetry.addLine("Share : Subtract 1 second");
        telemetry.addData("Delay", delay + " seconds");

        telemetry.addData("Heading", getHeading());
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
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

        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;

        //Checks
        UpdateIntake();
        UpdateWait();
        Shoot(false);


        // Test Mecanums
        /*if (step==0){
            MoveMecanum(305, 305, 0, 0.5);
            step++;
        }
        else if(step == 1 && mecanumMoveDone()){
            MoveMecanum(0, 0, 90, 0.5);
            step++;
        }*/

        // Starting side
        if (StartingSide == StartingSide.GOAL){
            //Steps here
            if(step == 0){ // Delay code for alliance
                Wait(delay);
                step++;
            }
            else if(step == 1 && !timerRunning){ //shoot 1
                Shoot(true);
                step++;
            }
            else if(step == 2){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0.5);
                    step++;
                }
            }
            else if(step == 3 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 4 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 5 && !timerRunning){ // shoot 2
                Shoot(true);
                step++;
            }
            else if(step == 6){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0.2);
                    step++;
                }
            }
            else if(step == 7 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 8 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 9 && !timerRunning){ // shoot 3
                Shoot(true);
                step++;
            }
            else if(step == 10){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0);
                    step++;
                }
            }
            else if(step == 11 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 12 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 13 && !timerRunning){ // shoot 4
                Shoot(true);
                step++;
            }
            else if(step == 14){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0);
                    step++;
                }
            }
            else if(step == 15 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 16 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 17 && !timerRunning){ // shoot 5
                Shoot(true);
                step++;
            }
            else if(step == 18){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0);
                    step++;
                }
            }
            else if(step == 19 && !timerRunning){ // Move Back
                flywheel.setVelocity(0);
                MoveMecanum(0, -305, 0, 0.7);
                step++;
            }
            else if (step == 20 && mecanumMoveDone()) { // Wait
                Wait(0.1);
                step++;
            }
            else if (step == 21 && !timerRunning){ // Move Strafe Out
                MoveMecanum(350 * (StartingColor.sign), 0, 0, 0.5);
                step++;
            }
            else if (step == 22 && mecanumMoveDone()){ // End code
                requestOpModeStop();
            }
        }
        else if (StartingSide == StartingSide.TRIANGLE){ // Triangle Launch //////////////////////////////////////////////
            if(step == 0){// Delay code for alliance
                Wait(delay);
                step++;
            }
            else if (step == 1 && !timerRunning){// Strafe 1ft out the zone
                MoveMecanum(0, 500, 0, 0.7);
                step++;
            }
            else if (step == 2 && mecanumMoveDone()){ // End code
                requestOpModeStop();
            }
        }
        else { // Triangle to Goal /////////////////////////////////////////////////////////
            if(step == 0){ // Delay code for alliance
                Wait(delay);
                step++;
            }
            else if (step == 1 && !timerRunning){ //Move Forward
                MoveMecanum(0, 1300, 0, 0.7);
                step++;
            }
            else if(step==2 && mecanumMoveDone()){//Turn
                MoveMecanum(0, 0, 45 * (StartingColor.sign), 0.4);
                step++;
            }
            else if(step == 3 && mecanumMoveDone()){// Towards Goal
                MoveMecanum(0, 950, 0, 0.4);
                step++;
            }
            else if(step == 4 && mecanumMoveDone()){ //req shot 1
                Shoot(true);
                step++;
            }
            else if(step == 5){ // wait until finished
                if (LiftState == LiftState.IDLE){
                    Wait(0.5);
                    step++;
                }
            }
            else if(step == 6 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 7 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 8 && !timerRunning){ // shoot 2
                Shoot(true);
                step++;
            }
            else if(step == 9){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0.2);
                    step++;
                }
            }
            else if(step == 10 && !timerRunning){ // Intake
                Intake(1);
                step++;
            }
            else if (step == 11 && !intakeRunning){ // Wait
                Wait(0.5);
                step++;
            }
            else if (step == 12 && !timerRunning){ // shoot 3
                Shoot(true);
                step++;
            }
            else if(step == 13){
                if (LiftState == LiftState.IDLE){ // Wait until finished shot
                    Wait(0.2);
                    step++;
                }
            }
            else if(step == 14 && !timerRunning){
                MoveMecanum(305 * (StartingColor.sign), -305, 0, 0.5);
                step++;
            }
            else if(step == 15 && mecanumMoveDone()){
                requestOpModeStop();
            }
        }


        // Telemetry
        telemetry.addData("Spinning: ", spinning);
        telemetry.addData("Lift:", lifting);
        telemetry.addData("Flywheel RPM: ", flywheel.getVelocity());
        telemetry.addData("Lift: ", lift.getPosition());
        telemetry.addData("Step:", step);
        telemetry.addData("Heading", getHeading());
        telemetry.update();
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    STOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void stop() {
        telemetry.addData("Time to complete :", timeToComplete.seconds());
        telemetry.update();
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    FUNCTIONS
    /////////////////////////////////////////////////////////////////////////////////////*/
    int mmToTicks(double distanceMM){
        return (int)((distanceMM/wheelCircumference) * TICKS_PER_REV * GEAR_RATIO);
    }

    //Check Functions
    double getHeading(){
        Orientation angles = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        return angles.firstAngle; // yaw (heading)
    }
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

    // Moving Functions
    void Wait(double time){
        waitTimer.reset();
        timerDuration = time;
        timerRunning = true;
    }

    void MoveMecanum(double xMM, double yMM, double rotationDeg, double power){


        //Convert rotation from degrees to approximate wheel linear distance (arc length)
        double turnRadiusMM = Math.hypot(ROBOT_LENGTH_MM / 2.0, ROBOT_WIDTH_MM / 2.0);
        double rotationMM = Math.toRadians(rotationDeg) * turnRadiusMM;

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
