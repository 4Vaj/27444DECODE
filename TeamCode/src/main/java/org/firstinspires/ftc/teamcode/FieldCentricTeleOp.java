package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric TeleOp", group = "Aveion")
//@Disabled
public class FieldCentricTeleOp extends OpMode{

    //Timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime conveyTimer = new ElapsedTime();
    ElapsedTime waitTimer = new ElapsedTime();

    //Independent Variables
    final double pushPos = 0.65;

    final double pushPosSet = 0.38;
    final double liftTime = 0.1;

    final double conveyTime = 2;
    final double conveyWaitTime = 0.25;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Drive Gears
    final double gearOne = 0.4;
    final double gearTwo = 0.6;
    final double gearThree = 1;

    // Target Velocities
    final double tolerance = 50;
    final double VelocityOne = 1100;
    final double VelocityTwo = 1500;
    final double VelocityThree = 2000;
    final double VelocityFour = 2500;

    //Dependent Variables
    double manualHoodAdjust = 0;

    // Launcher Target and Minimumm Velocities
    private double LAUNCHER_TARGET_VELOCITY = 0;
    private double LAUNCHER_MIN_VELOCITY = 0;
    private double gear = 0;
    private String lifting = null;
    private boolean spinning;
    private boolean conveying;
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
    //private LaunchState launchState;
    private LiftState LiftState;
    private MLiftState MLiftState;

    //Variables for each drive motor
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;


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
        LAUNCHER_TARGET_VELOCITY = VelocityOne;
        LAUNCHER_MIN_VELOCITY = VelocityOne - tolerance;
        spinning = false;
        conveying = false;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        gear = gearTwo;
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {
        //Mecanum Drive
        driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //Velocities
        if (gamepad2.dpad_left){
            LAUNCHER_TARGET_VELOCITY = VelocityOne;
        }
        else if (gamepad2.dpad_up){
            LAUNCHER_TARGET_VELOCITY = VelocityTwo;
        }
        else if (gamepad2.dpad_right){
            LAUNCHER_TARGET_VELOCITY = VelocityThree;
        }
        else if (gamepad2.dpad_down){
            LAUNCHER_TARGET_VELOCITY = VelocityFour;
        }
        else if (gamepad2.rightBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY += 50;
        }
        else if (gamepad2.leftBumperWasPressed()){
            LAUNCHER_TARGET_VELOCITY -= 50;
        }

        LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 200;


        //Manual Controls
        if (gamepad2.cross) {
            spinning = true;
            flywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }

        else if (gamepad2.circle) {
            spinning = false;
            flywheel.setZeroPowerBehavior(BRAKE);
            flywheel.setPower(0);
            flywheel.setZeroPowerBehavior(FLOAT);
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

        //Hood Controls
        if (gamepad1.left_bumper)
        {
            manualHoodAdjust -= 0.05;
            if (manualHoodAdjust < 0) {
                manualHoodAdjust = 0;
            }
        }
        else if (gamepad1.right_bumper)
        {
            manualHoodAdjust += 0.05;
            if (manualHoodAdjust > 1) {
                manualHoodAdjust = 1;
            }
        }
        hood.setPosition(manualHoodAdjust);


        leftIntake.setPower(-gamepad2.left_trigger);
        rightIntake.setPower(-gamepad2.left_trigger);
        leftConveyor.setPower(-gamepad2.left_trigger);
        rightConveyor.setPower(-gamepad2.left_trigger);

        if(gamepad2.square){
            leftIntake.setPower(1);
            rightIntake.setPower(1);
            leftConveyor.setPower(1);
            rightConveyor.setPower(1);
        }
//
        //Auto controls
        Shoot((gamepad2.triangleWasPressed()));
        ManualLift(gamepad2.right_trigger > 0.5);
        telemetry.addData("Spinning", spinning);
        telemetry.addData("Lift", lifting);
        telemetry.addData("Conveying", conveying);
        telemetry.addData("Flywheel RPM", flywheel.getVelocity());
        telemetry.addData("Lift", lift.getPosition());

        //telemetry.addLine("12345678912345678912345678912345678912");
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("D-pad left", VelocityOne);
        telemetry.addData("D-pad up", VelocityTwo);
        telemetry.addData("D-pad down", VelocityThree);
        telemetry.addData("D-pad right", VelocityFour);
        telemetry.addData("Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("Drive Power", gear);
        telemetry.addLine("----------------------------------------------------------------------------");
        telemetry.addData("Hood Position", manualHoodAdjust);
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

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        mecanumDrive(newForward, newRight, rotate);
    }
    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        frontLeftPower = (( forward  + strafe + rotate) * gear) / denominator;
        frontRightPower = ((forward  - strafe - rotate) * gear) / denominator;
        backLeftPower = ((  forward  - strafe + rotate) * gear ) / denominator;
        backRightPower = (( forward  + strafe - rotate) * gear ) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

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
