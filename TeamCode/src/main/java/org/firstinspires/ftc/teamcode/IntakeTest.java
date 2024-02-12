package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="IntakeTest", group="Linear Opmode")

public class IntakeTest extends LinearOpMode {
    DcMotor intakeMotor = null;
    double intakePower;
    double leftY;
    double motorMax = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intakeMotor = hardwareMap.dcMotor.get("intake");

        initMotors();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // controller 1
            leftY = -gamepad1.left_stick_y; // Remember, this is reversed!


            intakePower = leftY * motorMax;

            intakeMotor.setPower(intakePower);

            telemetry.addData("LF", "%.3f", intakePower);

        }
    }

    public void initMotors(){
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
