package org.firstinspires.ftc.teamcode;

// Imports

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AveionTeleOpLaunchTest", group = "Aveion")
//@Disabled
public class AveionTeleOpLaunchTest extends OpMode{

    //Timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();

    //Independent Variables
    final double pushPosL = 0;
    final double pushPosR = 0.18;

    final double pushPosLSet = 0.2;
    final double pushPosRSet = 0;
    final double liftTime = 1;
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Launcher Target and Min Velocities
    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MIN_VELOCITY = 1100;

    //Dependent Variables

    private String lifting = null;
    private boolean spinning;
    //Declare Motors
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotorEx launcher1 = null;
    private DcMotorEx launcher2 = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private CRServo leftConveyor = null;
    private CRServo rightConveyor = null;
    private Servo leftPusher = null;
    private Servo rightPusher = null;


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
    private LaunchState launchState;
    private LiftState LiftState;

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
        launchState = LaunchState.IDLE;
        LiftState = LiftState.IDLE;

        //Configuration
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launch2");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeed");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeed");
        leftConveyor = hardwareMap.get(CRServo.class, "leftConvey");
        rightConveyor = hardwareMap.get(CRServo.class, "rightConvey");
        leftPusher = hardwareMap.get(Servo.class, "leftLift");
        rightPusher = hardwareMap.get(Servo.class, "rightLift");

        //Motor Directions

            //Drive Motors
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            //Launch Motors
            launcher1.setDirection(DcMotorEx.Direction.FORWARD);
            launcher2.setDirection(DcMotorEx.Direction.REVERSE);

        //Servo Directions

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        //Motor Behaviors

            //Launch Motors
            launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Drive Motors
            frontRight.setZeroPowerBehavior(BRAKE);
            frontLeft.setZeroPowerBehavior(BRAKE);
            backRight.setZeroPowerBehavior(BRAKE);
            backLeft.setZeroPowerBehavior(BRAKE);

        //Servo Behaviors

            //Feeders
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);

        //PIDF Coefficients (i actually don't know how these work)
        launcher1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    BEFORE START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void init_loop() {
        //Set push servos to 0 position
    }
    /*//////////////////////////////////////////////////////////////////////////////////////
    START
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void start() {
        spinning = false;
        lifting = "";
    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {
        //Mecanum Drive
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if (gamepad1.square){
            spinning = true;
            launcher1.setVelocity(1200);
            launcher2.setVelocity(1200);
        }

        else if (gamepad1.triangle){
            spinning = false;
            launcher1.setVelocity(0);
            launcher2.setVelocity(0);
        }


        //Lift(gamepad1.rightBumperWasPressed());


        telemetry.addData("Spinning: ", spinning);
        telemetry.addData("Lift:", lifting);

        telemetry.addData("Left Motor RPM: ", launcher1.getVelocity());
        telemetry.addData("Right Motor RPM: ", launcher2.getVelocity());


        telemetry.addData("Lift L: ", leftPusher.getPosition());
        telemetry.addData("Lift R: ", rightPusher.getPosition());
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
    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        frontLeftPower = (forward + strafe + rotate) / denominator;
        frontRightPower = (forward - strafe - rotate) / denominator;
        backLeftPower = (forward - strafe + rotate) / denominator;
        backRightPower = (forward + strafe - rotate) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

//    void Lift(boolean shotRequested) {
//        switch (LiftState){
//            case IDLE:
//                spinning = false;
//                if (shotRequested){
//                    LiftState = LiftState.SPOOL;
//                }
//                break;
//            case SPOOL:
//                spinning = true;
//                launcher1.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                launcher2.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                if (launcher1.getVelocity() > LAUNCHER_MIN_VELOCITY && launcher2.getVelocity() > LAUNCHER_MIN_VELOCITY)
//                {
//                    LiftState = LiftState.LIFT;
//                }
//                break;
//            case LIFT:
//                lifting = "Lifting";
//                leftPusher.setPosition(pushPosL);
//                rightPusher.setPosition(pushPosR);
//                liftTimer.reset();
//                LiftState = LiftState.LIFTING;
//                break;
//            case LIFTING:
//                if (liftTimer.seconds() > liftTime) {
//                    lifting = "";
//                    leftPusher.setPosition(pushPosLSet);
//                    rightPusher.setPosition(pushPosRSet);
//                    LiftState = LiftState.IDLE;
//                }
//                break;
//        }
//    }

    /*void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPOOL;
                }
                break;
            case SPOOL:
                launcher1.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher1.getVelocity() > LAUNCHER_MIN_VELOCITY) {
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
