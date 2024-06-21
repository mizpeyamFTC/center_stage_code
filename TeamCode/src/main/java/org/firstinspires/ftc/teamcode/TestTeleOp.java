package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor elevatorRight, elevatorLeft;
    private DcMotor leftArmMotor, rightArmMotor;
    private CRServo finalArmJoint;

    private Servo leftClawServo, rightClawServo;

    private boolean leftClawOpen = true;
    private boolean rightClawOpen = true;
    private Servo planeServo;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU controlHubIMU;
    private DistanceSensor leftClawSensor;
    private DistanceSensor rightClawSensor;
    private double DIST_NORM, SIDE_DIST_NORM, WHEEL_DIAMETER_CM, WHEEL_BASE_DISTANCE, FULL_ROUND;
    private double COUNTS_PER_CM;
    private ElapsedTime runtime;
    private ElapsedTime matchRuntime;
    private double motorMax = 0.5;
    private int newLeftFrontTarget;
    private int newRightFrontTarget;
    private int newLeftRearTarget;
    private int newRightRearTarget;
    private int sleepTime = 30;
    private double targetHeading = 0;
    private double headingError = 0;
    private double turnSpeed = 0.6;
    private int intakeMotorPositionCount;
    private int middleArmJointHomePosition;

    private final double CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private final double CORE_HEX_DEGREES_PER_COUNT = 360 / CORE_HEX_COUNTS_PER_REVOLUTION;
    private final double COUNTS_PER_MOTOR_REV = 28;    //כמות צעדים עבור סיבוב מנוע
    private final double DRIVE_GEAR_REDUCTION = 20.0;     // יחס גירים
    private final double GOB_WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    private final double REV_WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    private final double GOB_WHEEL_BASE_DISTANCE = 39;     // For figuring circumference
    private final double REV_WHEEL_BASE_DISTANCE = 37;     // For figuring circumference
    private final double TURN_SPEED = 0.3;
    private final double MIN_TURN_SPEED = 0.2;
    private final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.
    private final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    private final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    private final int COLOR_THRESHOLD = 400; // blue
    private final double DRIVE_MAX = 0.9;
    private final double TURN_MAX = 0.5;
    private final boolean MAIN_ROBOT = true; // false for ROBOT_B
    private final int LEFT_CLAW_CLOSED = 0;
    private final int LEFT_CLAW_OPENED = 1;
    private final int RIGHT_CLAW_CLOSED = 1;
    private final int RIGHT_CLAW_OPENED = 0;
    private final long SHORT_SIDE_WAIT = 10000;

    private int LEFT_ARM_HOME_POSITION;
    private int RIGHT_ARM_HOME_POSITION;
    private int LEFT_ARM_UP_POSITION;
    private int RIGHT_ARM_UP_POSITION;
    private int LEFT_ARM_SCORE_POSITION;
    private int RIGHT_ARM_SCORE_POSITION;
    private int LEFT_ARM_INTAKE_POSITION;
    private int RIGHT_ARM_INTAKE_POSITION;
    private int LEFT_HOME_TO_UP_OFFSET;
    private int RIGHT_HOME_TO_UP_OFFSET;
    private int LEFT_HOME_TO_SCORE_OFFSET;
    private int RIGHT_HOME_TO_SCORE_OFFSET;
    private int LEFT_HOME_TO_INTAKE_OFFSET;
    private int RIGHT_HOME_TO_INTAKE_OFFSET;


    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    Thread clawDistanceSensorThread;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        clawDistanceSensorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()&&!isStopRequested()){
                    try {
                        closeClawsOnPixelDetection();

                    } catch (Exception e){
                        e.printStackTrace();

                    }
                }
            }
        });


        while (opModeIsActive()){
            driveByInput();
            if(gamepad2.x) openLeftClaw();//almost done
            if(gamepad2.a) openRightClaw();//almost done

            if(gamepad2.left_bumper) closeLeftClaw();
            if(gamepad2.right_bumper) closeRightClaw();
            if(gamepad1.a) resetIMU();
            if(gamepad2.ps){
                openLeftClaw();
                openRightClaw();
            }
            updateArmTelemetry();

            elevatorLeft.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            elevatorRight.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
            gp2LeftStickY = gamepad2.left_stick_y;
            gp2LeftStickX = gamepad2.left_stick_x;
            gp2RightStickY = gamepad2.right_stick_y;
            gp2RightStickX = gamepad2.right_stick_x;


            finalArmJoint.setPower(gp2LeftStickY*-1);
            rightArmMotor.setPower(gp2RightStickY*0.7);
            leftArmMotor.setPower(gp2RightStickY*0.7);

            if(gamepad1.dpad_up){
                leftFront.setPower(1);
                telemetry.addLine("dpad up = left front forward");
            }if(gamepad1.dpad_left){
                leftRear.setPower(1);
                telemetry.addLine("dpad left = left rear forward");
            }if(gamepad1.dpad_right){
                rightFront.setPower(1);
                telemetry.addLine("dpad right = right front forward");
            }if(gamepad1.dpad_down){
                rightRear.setPower(1);
                telemetry.addLine("dpad down = right rear forward");
            }

            //clawDistanceSensorThread.start();

            //closeClawsOnPixelDetection();

        }
    }
    private void resetIMU(){
        initIMU();
    }

    private void closeClawsOnPixelDetection(){
        if(leftClawSensor.getDistance(DistanceUnit.CM)<5) closeLeftClaw();
        if(rightClawSensor.getDistance(DistanceUnit.CM)<5) closeRightClaw();

    }



    private void updateArmTelemetry()
    {
        telemetry.addLine("left motor pos:" + leftArmMotor.getCurrentPosition());
        telemetry.addLine("right motor pos:" + rightArmMotor.getCurrentPosition());
        telemetry.update();

    }
    private void driveByInput(){
        double denominator, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
        double rotX, rotY;
        double botHeading;
        double gp2LeftStickY, gp2LeftStickX, gp2RightStickY, gp2RightStickX;
        double y, x,rx;
        double maxOutputPower;
        gp2LeftStickY = gamepad2.left_stick_y;
        gp2LeftStickX = gamepad2.left_stick_x;
        gp2RightStickY = gamepad2.right_stick_y;
        gp2RightStickX = gamepad2.right_stick_x;

        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        motorMax = 0.5+gamepad1.right_trigger*0.5;

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
    private void positionToIntake(){}
    private void positionToScore(){

    }
    private void positionToRest(){}
    private void openLeftClaw(){
        leftClawServo.setPosition(LEFT_CLAW_OPENED);
    }
    private void closeLeftClaw(){
        leftClawServo.setPosition(LEFT_CLAW_CLOSED);
    }
    private void openRightClaw(){
        rightClawServo.setPosition(RIGHT_CLAW_OPENED);
    }
    private void closeRightClaw(){
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
    }

    private void initializeRobot() {
        //initCamera();
        initRobotParameters();
        initMotors();
        initIMU();
        closeRightClaw();
        closeLeftClaw();
    }
    private void initRobotParameters() {
        runtime = new ElapsedTime();
        matchRuntime = new ElapsedTime();
        initDriveConstants();
    }

    private void initDriveConstants() {
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
    private void initArmConstants(){
        LEFT_ARM_HOME_POSITION = leftArmMotor.getCurrentPosition();
        RIGHT_ARM_HOME_POSITION = rightArmMotor.getCurrentPosition();

        LEFT_ARM_UP_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_UP_OFFSET;
        RIGHT_ARM_UP_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_UP_OFFSET;

        LEFT_ARM_SCORE_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_SCORE_OFFSET;
        RIGHT_ARM_SCORE_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_SCORE_OFFSET;

        LEFT_ARM_INTAKE_POSITION = LEFT_ARM_HOME_POSITION + LEFT_HOME_TO_INTAKE_OFFSET;
        RIGHT_ARM_INTAKE_POSITION = RIGHT_ARM_HOME_POSITION + RIGHT_HOME_TO_INTAKE_OFFSET;
    }

    private void initIMU() {
        // Retrieve the IMU from the hardware map
        controlHubIMU = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        controlHubIMU.initialize(parameters);
        controlHubIMU.resetYaw();


    }
    private void initMotors(){
        assignHardware();
        //intakeMotor = hardwareMap.dcMotor.get("intake");
        // liftMotor = hardwareMap.dcMotor.get("lift");
        initMotorsDirection();
        resetAllMotorsEncoders();
        setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void assignHardware() {


        leftFront = hardwareMap.dcMotor.get("leftFront");//color: blank, ExpansionHub, port:2
        rightFront = hardwareMap.dcMotor.get("rightFront");//color: blank, ControlHub, port:2
        leftRear = hardwareMap.dcMotor.get("leftRear");//color: blank, ExpansionHub, port:3
        rightRear = hardwareMap.dcMotor.get("rightRear");//color: blank, ControlHub, port:3
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");//color: RED, ExpansionHub, port:1
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");//color: BLUE, ControlHub, port:1
        finalArmJoint = hardwareMap.get(CRServo.class,"finalArmJoint" );//color: RED-BLUE, ControlHub, port:0
        leftClawServo = hardwareMap.get(Servo.class,"leftClawServo" );//color: RED, ExpansionHub, port:0
        rightClawServo = hardwareMap.get(Servo.class,"rightClawServo" );//color: BLUE, ControlHub, port:0
        planeServo = hardwareMap.get(Servo.class, "planeServo");//color: blank, ControlHub, port:0
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");//color: blank, ControlHub, port:0
        //leftClawSensor = hardwareMap.get(DistanceSensor.class, "leftClawSensor");//color: blank, ControlHub, port:0
        //rightClawSensor = hardwareMap.get(DistanceSensor.class, "rightClawSensor");//color: blank, ControlHub, port:0
        elevatorRight = hardwareMap.dcMotor.get("elevatorRight");//color: BLUE, ControlHub, port:0
        elevatorLeft = hardwareMap.dcMotor.get("elevatorLeft");//color: RED, ExpansionHub, port:0

    }
    private void resetAllMotorsEncoders() {
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setAllMotorsMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
        elevatorRight.setMode(runMode);
        elevatorLeft.setMode(runMode);
    }
    private void initMotorsDirection() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);
        finalArmJoint.setDirection(CRServo.Direction.FORWARD);
        elevatorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void turnOnRunToPosition() {
        setAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    //*************************INITIALIZE*************************
}

