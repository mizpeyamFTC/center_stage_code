package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="TeleOp1", group="Linear Opmode")

public class TeleOp1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront, leftRear, rightFront, rightRear, intakeMotor, liftMotor;
    private  DcMotor firstArmJoint, middleArmJoint;
    private CRServo finalArmJoint;
    private Servo clawServo;
    //finalArmJoint, clawServo;
    private double LF, LR, RF, RR;
    private double leftX, leftY, rightX, rightY;
    double motorMax = 0.5;
    double intakePow, liftPow = 0.2;
    IMU imu;
    boolean isFinal = true;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    private final double CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private final double CORE_HEX_DEGREES_PER_COUNT = 360/CORE_HEX_COUNTS_PER_REVOLUTION;

    private final double HOME_TO_INTAKE_DEGREES = 20;

    private int intakeMotorPositionCount;
    private int middleArmJointHomePosition;
    private boolean firstArmDeployment = false;
    @Override
    public void runOpMode() {


        initRobot();
        // Declare our motors
        // Make sure your ID's match your configuration


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        if(opModeIsActive()){
            fieldOrientedWhile();
        }

    }




    private void fieldOrientedWhile(){
         double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double rotX, rotY;
        double botHeading;
        double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
        double y, x,rx;
        while (opModeIsActive()) {
            updateTelemetryRunTime();



            //********************* EXTRACT TO updateTelemetry
            updateTelemetry();
            //*********************

            gp2LeftStickY = gamepad2.left_stick_y;
            gp2LeftStickX = gamepad2.left_stick_x;
            gp2RightStickY = gamepad2.right_stick_y;
            gp2RightStickX = gamepad2.right_stick_x;

            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            motorMax = 0.7+gamepad1.right_trigger/10*3;


            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetryAddIMUData();
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = motorMax* (rotY + rotX + rx) / denominator;
            backLeftPower = motorMax*(rotY - rotX + rx) / denominator;
            frontRightPower = motorMax*(rotY - rotX - rx) / denominator;
            backRightPower = motorMax*(rotY + rotX - rx) / denominator;

            powerDriveMotors(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            /**
             * game pad 2:
             * a = move arm for intake
             * y = move arm for lift transaction
             * b = open claw
             * x = close claw
             * RT = lift up
             * LT = lift down
             * left joystick = final arm joint
             * right joystick = middle arm joint
             * RB = lift height 1
             * middle logitech button = lift height 2
             * LB = lift height 3
             * Dpad left = left AprilTag
             * Dpad up = middle AprilTag
             * Dpad right = right AprilTag
             */

            if(gamepad2.a) intake();
            if(gamepad2.y) transferToLift();
            if(gamepad2.b) openClaw();//almost done
            if(gamepad2.x) closeClaw();//almost done
            if(gamepad2.right_bumper)  scoreLowHeight();
            if(gamepad2.touchpad) scoreMiddleHeight();
            if(gamepad2.left_bumper)  scoreHighHeight();
            if(gamepad2.dpad_left) centerOnLeftAprilTag();//optional
            if(gamepad2.dpad_up) centerOnMiddleAprilTag();//optional
            if(gamepad2.dpad_right) centerOnRightAprilTag();//optional
            if(gamepad1.a) resetIMU();//optional


            finalArmJoint.setPower(gp2LeftStickY*0.8);
            middleArmJoint.setPower(gp2RightStickY*0.8);

          /*
            if(gamepad2.right_trigger > 0.5) {
                //liftMotor.setPower(liftPow);
                int liftPosition;
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPosition = liftMotor.getCurrentPosition();
                liftMotor.setTargetPosition(liftPosition+200);
            }
            if(gamepad2.left_trigger > 0.5) {
                //liftMotor.setPower(-liftPow);
                int liftPosition;
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPosition = liftMotor.getCurrentPosition();
                liftMotor.setTargetPosition(liftPosition-200);
            }
            if(gamepad1.a){
                int aprilTagId =4;
                int distanceCM =10;
                RunToId(aprilTagId, distanceCM);
            }

           */

        }

    }
    private void resetIMU(){
        imu.resetYaw();

    }

    private void powerDriveMotors(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    private void initRobot() {

        initTelemetry();
        initMotors();
        initIMU();
        //initCamera();
        waitForStart();
        runtime.reset();
    }

    private void telemetryAddIMUData() {
        telemetry.addData("yaw:",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) );
        telemetry.addData("pitch:",imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) );
        telemetry.addData("roll:",imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) );
        telemetry.update();
    }

    private void centerOnRightAprilTag() {
    }

    private void centerOnMiddleAprilTag() {

    }

    private void centerOnLeftAprilTag() {

    }

    private void scoreHighHeight() {

    }

    private void scoreMiddleHeight() {

    }

    private void scoreLowHeight() {

    }

    private void transferToLift() {
        closeClaw();
        orientMiddleArmForLiftTransaction();
        orientFinalJointForLiftTransaction();

    }

    private void orientFinalJointForLiftTransaction() {
    }

    private void orientMiddleArmForLiftTransaction() {
        middleArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleArmJoint.setTargetPosition(middleArmJointHomePosition);
        middleArmJoint.setPower(-0.7);//check if correct
    }

    private void intake() {
        openClaw();
        orientMiddleArmForIntake();
        orientFinalJointForIntake();


    }
    private void orientMiddleArmForIntake(){
        int targetPosition = intakeMotorPositionCount;
        middleArmJoint.setTargetPosition(targetPosition);
        middleArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleArmJoint.setPower(0.7);


    }

    private void orientFinalJointForIntake() {

    }

    private void closeClaw() {
        clawServo.setPosition(0);
    }

    private void openClaw() {
        clawServo.setPosition(1);
    }


    private void RunToId(int aprilTagId, int distanceCM) {



    }

    private void powerMotors() {
        leftFront.setPower(LF);
        rightFront.setPower(RF);
        leftRear.setPower(LR);
        rightRear.setPower(RR);
        //intakeMotor.setPower(intakePow);
        liftMotor.setPower(liftPow);
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
        telemetry.addData("LIFT", "%.3f", liftPow);

        /*if(tagProcessor.getDetections().size()>0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            telemetry.addData("x",tag.ftcPose.x);
            telemetry.addData("y",tag.ftcPose.y);
            telemetry.addData("z",tag.ftcPose.z);
            telemetry.addData("roll",tag.ftcPose.roll);
            telemetry.addData("pitch",tag.ftcPose.pitch);
            //telemetry.addData("yaw",tag.ftcPose.yaw);
            telemetry.addData("tag id:",tag.id);
            telemetry.update();
        }

         */

    }
    private void addDataToTelemetry(String caption, String format, String data ){
        telemetry.addData(caption, format, data);
        telemetry.update();
    }

    private void updateTelemetryRunTime() {
        telemetry.addData("Status", "Run Timed: " + runtime.toString());
        telemetry.update();
    }




    private void initIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    private void initCamera(){
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }
    private void initTelemetry() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    private void initMotors(){
        assignHardwareToMotors();

        //intakeMotor = hardwareMap.dcMotor.get("intake");
       // liftMotor = hardwareMap.dcMotor.get("lift");

        initMotorsDirection();

        resetAllMotorsEncoders();

        setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        middleArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firstArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    private void assignHardwareToMotors() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        firstArmJoint = hardwareMap.get(DcMotor.class, "firstArmJoint");
        middleArmJoint = hardwareMap.get(DcMotor.class, "middleArmJoint");
        finalArmJoint = hardwareMap.get(CRServo.class,"finalArmJoint" );
        clawServo = hardwareMap.get(Servo.class,"clawServo" );
    }

    private void resetAllMotorsEncoders() {
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setAllMotorsMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
        firstArmJoint.setMode(runMode);
        middleArmJoint.setMode(runMode);
    }



    private void initMotorsDirection() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        firstArmJoint.setDirection(DcMotor.Direction.FORWARD);
        middleArmJoint.setDirection(DcMotor.Direction.REVERSE);
        finalArmJoint.setDirection(CRServo.Direction.REVERSE);
    }

}
