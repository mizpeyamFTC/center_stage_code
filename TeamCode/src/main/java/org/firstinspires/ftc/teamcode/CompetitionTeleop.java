package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "CompetitionTeleop")
public class CompetitionTeleop extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private  DcMotor middleArmJoint;
    private CRServo finalArmJoint;
    private Servo clawServo;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU controlHubIMU;

    private final boolean MAIN_ROBOT = true; // false for ROBOT_B
    private double DIST_NORM, SIDE_DIST_NORM, WHEEL_DIAMETER_CM, WHEEL_BASE_DISTANCE, FULL_ROUND;
    private double COUNTS_PER_CM;

    private ElapsedTime runtime;
    private ElapsedTime matchRuntime;

    private double motorMax = 0.5;

    private final double driveMax = 0.9;
    private final double turnMax = 0.5;


    private final long SHORT_SIDE_WAIT = 10000;

    private int newLeftFrontTarget;
    private int newRightFrontTarget;
    private int newLeftRearTarget;
    private int newRightRearTarget;
    private int sleepTime = 30;
    private double targetHeading = 0;
    private double headingError = 0;
    private double turnSpeed= 0.6;
    private int intakeMotorPositionCount;
    private int middleArmJointHomePosition;

    private final double CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private final double CORE_HEX_DEGREES_PER_COUNT = 360/CORE_HEX_COUNTS_PER_REVOLUTION;
    private final double COUNTS_PER_MOTOR_REV = 28;    //כמות צעדים עבור סיבוב מנוע
    private final double DRIVE_GEAR_REDUCTION = 20.0;     // יחס גירים
    private final double GOB_WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    private final double REV_WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    private final double GOB_WHEEL_BASE_DISTANCE = 39;     // For figuring circumference
    private final double REV_WHEEL_BASE_DISTANCE = 37;     // For figuring circumference
    private final double TURN_SPEED= 0.3;
    private final double MIN_TURN_SPEED = 0.2;
    private final double HEADING_THRESHOLD= 0.5 ;    // How close must the heading get to the target before moving to next step.
    private final double P_DRIVE_GAIN= 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    private final double P_TURN_GAIN= 0.02;     // Larger is more responsive, but also less stable
    private final int COLOR_THRESHOLD = 400; // blue


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initializeRobot();

        double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double rotX, rotY;
        double botHeading;
        double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
        double y, x,rx;
        double maxOutputPower;

        while(opModeIsActive()){

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

            botHeading = controlHubIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetryAddIMUData();
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            maxOutputPower = motorMax / denominator;
            frontLeftPower =maxOutputPower* (rotY + rotX + rx);
            backLeftPower = maxOutputPower*(rotY - rotX + rx);
            frontRightPower = maxOutputPower*(rotY - rotX - rx);
            backRightPower = maxOutputPower*(rotY + rotX - rx);

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
            if(gamepad1.a) resetIMU();



            finalArmJoint.setPower(gp2LeftStickY*0.8);
            middleArmJoint.setPower(gp2RightStickY*0.8);





        }



    }

    private void updateTelemetry() {
        /*
        telemetry.addData("LF", "%.3f", LF);
        telemetry.addData("RF", "%.3f", RF);
        telemetry.addData("LR", "%.3f", LR);
        telemetry.addData("RR", "%.3f", RR);
        telemetry.addData("LEFT Y", "%.3f", leftY);
        telemetry.addData("LEFT X", "%.3f", leftX);
        telemetry.addData("RIGHT Y", "%.3f", rightY);
        telemetry.addData("RIGHT X", "%.3f", rightX);
        telemetry.addData("LIFT", "%.3f", liftPow);

         */

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
    private void resetIMU(){
        initIMU();
    }

    private void powerDriveMotors(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
    private void telemetryAddIMUData() {
        telemetry.addData("yaw:", controlHubIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) );
        telemetry.addData("pitch:", controlHubIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) );
        telemetry.addData("roll:", controlHubIMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) );
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
    //************************* - INITIALIZE - *************************
    private void initializeRobot() {

        initRobotParameters();
        initMotors();
        initIMU();

    }
    private void initRobotParameters() {
        runtime = new ElapsedTime();
        matchRuntime = new ElapsedTime();
        boolean bLedOn = true;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");
        colorSensor = hardwareMap.get(ColorSensor.class, "cs-1");
        colorSensor.enableLed(bLedOn);
        if (MAIN_ROBOT) {
            WHEEL_DIAMETER_CM = GOB_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = GOB_WHEEL_BASE_DISTANCE;
            DIST_NORM = 1;
            SIDE_DIST_NORM = 1.15;
        }
        else {
            WHEEL_DIAMETER_CM =REV_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = REV_WHEEL_BASE_DISTANCE;
            DIST_NORM = 0.9;
            SIDE_DIST_NORM =0.9;

        }
        COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_CM * 3.1415);
        FULL_ROUND = COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION*(WHEEL_BASE_DISTANCE/WHEEL_DIAMETER_CM)*1.65;
    }
    private void initIMU() {
        // Retrieve the IMU from the hardware map
        controlHubIMU = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        controlHubIMU.initialize(parameters);
        controlHubIMU.resetYaw();


    }
    private void initMotors(){
        assignHardwareToMotors();

        //intakeMotor = hardwareMap.dcMotor.get("intake");
        // liftMotor = hardwareMap.dcMotor.get("lift");

        initMotorsDirection();

        resetAllMotorsEncoders();

        setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        middleArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private void assignHardwareToMotors() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        middleArmJoint = hardwareMap.get(DcMotor.class, "middleArmJoint");
        finalArmJoint = hardwareMap.get(CRServo.class,"finalArmJoint" );
        clawServo = hardwareMap.get(Servo.class,"clawServo" );
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");

    }
    private void resetAllMotorsEncoders() {
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setAllMotorsMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
        middleArmJoint.setMode(runMode);
    }
    private void initMotorsDirection() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        middleArmJoint.setDirection(DcMotor.Direction.REVERSE);
        finalArmJoint.setDirection(CRServo.Direction.REVERSE);
    }
    private void turnOnRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //*************************INITIALIZE*************************
}
