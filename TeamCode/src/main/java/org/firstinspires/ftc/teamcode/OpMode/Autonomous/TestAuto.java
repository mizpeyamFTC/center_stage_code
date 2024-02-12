
package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class TestAuto extends LinearOpMode {

    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    final double TICKS_PER_REV = 28;
    final double GEAR_RATIO = 20;
    final double WHEEL_DIAMETER_CM = 7.5;
    final double TICKS_PER_CM = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_CM * Math.PI);


    // gopower = speed..
    public double goPower =0.5;
    private int Sec = 1000;

    @Override
    public void runOpMode(){
        initMotors();
        waitForStart();
        startMove();
        //stopRobot();

    }


    private void  initMotors(){
        //map to robot
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        //reset encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set directions
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward (double distance){
        int target = (int)Math.round(distance*TICKS_PER_CM);
        leftFrontMotor.setTargetPosition(target + leftFrontMotor.getCurrentPosition());
        rightFrontMotor.setTargetPosition(target + rightFrontMotor.getCurrentPosition());
        leftRearMotor.setTargetPosition(target + leftRearMotor.getCurrentPosition());
        rightRearMotor.setTargetPosition(target + rightRearMotor.getCurrentPosition());

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /*private void moveLeft (double power, int time){
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);
        sleep (time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
    public void moveRight (double power, int time){
        moveLeft (-power, time);
    }

    public void moveBackwards (double power, int time){
        moveForward ( -power,  time);
    }*/

    private void startMove() {
        //int timesec = 2*Sec;
        moveForward(2000);
        /*moveForward(goPower,timesec);
        moveRight(goPower,timesec);
        moveBackwards(goPower,timesec);
        moveLeft(goPower,timesec);
        moveRight(goPower, 5*Sec);
        moveBackwards(goPower,3*Sec);
        moveRight(goPower,1.5*Sec);
        moveForward(goPower,1.5*Sec);
        moveLeft(goPower,1*Sec);
        moveForward(goPower,9.5*Sec);*/
    }

    private void stopRobot(){
        leftFrontMotor.setPower(10);
        rightFrontMotor.setPower(10);
        leftRearMotor.setPower(10);
        rightRearMotor.setPower(10     );
    }
}

