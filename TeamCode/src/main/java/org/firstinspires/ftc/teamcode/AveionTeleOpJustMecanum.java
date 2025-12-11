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

@TeleOp(name="AveionTeleOpJustMecanum", group = "Aveion")
//@Disabled
public class AveionTeleOpJustMecanum extends OpMode{
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Launcher Target and Min Velocities
    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MIN_VELOCITY = 1100;


    //Declare Motors
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPOOL,
        PUSH,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

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

        //Configuration
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        //Motor Directions

            //Drive Motors
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            //Launch Motors


        //Servo Directions



        //Motor Behaviors

            //Launch Motors


            //Drive Motors
            frontRight.setZeroPowerBehavior(BRAKE);
            frontLeft.setZeroPowerBehavior(BRAKE);
            backRight.setZeroPowerBehavior(BRAKE);
            backLeft.setZeroPowerBehavior(BRAKE);

        //Servo Behaviors

            //Feeders


        //PIDF Coefficients (i actually don't know how these work)

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

    }

    /*//////////////////////////////////////////////////////////////////////////////////////
    START LOOP BEFORE STOP
    /////////////////////////////////////////////////////////////////////////////////////*/

    @Override
    public void loop() {
        //Mecanum Drive
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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
