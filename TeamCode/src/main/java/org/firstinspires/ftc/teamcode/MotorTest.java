package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTes", group="Linear Opmode")

public class MotorTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private double LF, LR, RF, RR;
    private double leftX, leftY, rightX, rightY;
    double motorMax = 0.8;

    @Override
    public void runOpMode() {

        initRobot();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateTelemetryRunTime();

            // controller 1
            leftY = -gamepad1.left_stick_y; // Remember, this is reversed!
            leftX = gamepad1.left_stick_x; // Counteract imperfect strafing
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;

            LF = (leftY + leftX + rightX) * motorMax;
            LR = (leftY - leftX + rightX) * motorMax;
            RF = (leftY - leftX - rightX) * motorMax;
            RR = (leftY + leftX - rightX) * motorMax;

            //********************* - EXTRACT TO powerMotors()
            powerMotors();
            //*********************

            //********************* EXTRACT TO updateTelemetry
            updateTelemetry();
            //*********************

        }
    }

    private void powerMotors() {
        leftFront.setPower(LF);
        rightFront.setPower(RF);
        leftRear.setPower(LR);
        rightRear.setPower(RR);
    }

    private void updateTelemetry() {
        telemetry.addData("LF", "%.3f", LF);
        telemetry.addData("RF", "%.3f", RF);
        telemetry.addData("LR", "%.3f", LR);
        telemetry.addData("RR", "%.3f", RR);
        telemetry.addData("LEFT Y", "%.3f", leftY);
        telemetry.addData("LEFT X", "%.3f", leftX);
        telemetry.addData("RIGHT Y", "%.3f", rightY);
        telemetry.addData("RIGHT X", "%.3f", rightX);
        telemetry.update();
    }
    private void addDataToTelemetry(String caption, String format, String data ){
        telemetry.addData(caption, format, data);
        telemetry.update();
    }

    private void updateTelemetryRunTime() {
        telemetry.addData("Status", "Run Timed: " + runtime.toString());
        telemetry.update();
    }


    private void initRobot() {
        initTelemetry();
        initMotors();
        waitForStart();
        runtime.reset();
    }
    private void initTelemetry() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    public void initMotors(){

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
