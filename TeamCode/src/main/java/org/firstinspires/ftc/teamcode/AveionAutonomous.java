package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AveionSimpleAuto", group = "Aveion")
//@Disabled
public class AveionAutonomous extends OpMode{

    //Timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();

    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime waitTimer = new ElapsedTime();

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
    final double VelocityZero = 1000;
    final double VelocityOne = 1200;
    final double VelocityTwo = 1500;
    final double VelocityThree = 2000;
    final double VelocityFour = 2500;

    //Dependent Variables

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
        int countachSoundID = hardwareMap.appContext.getResources().getIdentifier("ountach", "raw", hardwareMap.appContext.getPackageName());
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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    INITIALIZE LOOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        // Pick a velocity to shoot at
        LAUNCHER_TARGET_VELOCITY = VelocityOne;
        LAUNCHER_MIN_VELOCITY = VelocityOne - 100;

        telemetry.addData("D-pad left:", VelocityOne);
        telemetry.addData("D-pad up:", VelocityTwo);
        telemetry.addData("D-pad down:", VelocityThree);
        telemetry.addData("D-pad right:", VelocityFour);
        telemetry.addLine("Please choose a velocity to run auto (default 1200).");
        telemetry.addData("Velocity:", LAUNCHER_TARGET_VELOCITY);
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        step = 0;
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {
        UpdateIntake();
        UpdateWait();
        //Steps here
        if(step == 0){
            Wait(5);
            step++;
        }
        else if(step == 1 && !timerRunning){

            Shoot(true);
            Intake(2);
            step++;
        }
        else if(step == 2 && !intakeRunning){
            Shoot(true);
            Intake(2);
            step++;
        }
        else if(step == 3 && !intakeRunning){
            Shoot(true);
            Intake(2);
            step++;
        }
        else if(step == 4 && !intakeRunning){
            MoveMecanum(0, -100, 0, 0.5);
            step++;
        }
        else if (step == 2 && mecanumMoveDone()) {
            Wait(1);
            step++;
        }
        else if (step == 3 && !timerRunning){
            MoveMecanum(100, 0, 0, 0.5);
            step++;
        }
        else if (step == 4 && mecanumMoveDone()){
            stop();
        }

        telemetry.addData("Spinning: ", spinning);
        telemetry.addData("Lift:", lifting);
        telemetry.addData("Flywheel RPM: ", flywheel.getVelocity());
        telemetry.addData("Lift: ", lift.getPosition());
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    STOP
    /////////////////////////////////////////////////////////////////////////////////////*/
    @Override
    public void stop() {
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

    void MoveMecanum(double xMM, double yMM, double rotationDeg, double power){
        //Convert rotation from degrees to approximate wheel linear distance (arc length)
        double rotationMM = Math.toRadians(rotationDeg) * ((ROBOT_LENGTH_MM + ROBOT_WIDTH_MM) / 2.0);

        // Calculate each wheel's linear distance (mm)
        double frontLeftMM  =  yMM + xMM + rotationMM;
        double frontRightMM =  yMM - xMM - rotationMM;
        double backRightMM   =  yMM - xMM + rotationMM;
        double backLeftMM  =  yMM + xMM - rotationMM;

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
