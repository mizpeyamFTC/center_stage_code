package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="LiftTest", group="Linear Opmode")

public class LiftTest extends LinearOpMode {
    DcMotor liftMotor = null;
    double liftPower;
    double leftY;
    double motorMax = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftMotor = hardwareMap.dcMotor.get("nhmnbrgl");

        initMotors();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // controller 1
            leftY = -gamepad1.left_stick_y; // Remember, this is reversed!


            liftPower = leftY * motorMax;

            liftMotor.setPower(liftPower);

            telemetry.addData("LF", "%.3f", liftPower);

        }
    }

    public void initMotors(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
