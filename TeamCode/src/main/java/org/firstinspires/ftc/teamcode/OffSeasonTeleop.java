package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class OffSeasonTeleop  extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor liftL, liftR;
    private DcMotor elevatorRight;
    private DcMotor elevatorLeft;
    private DcMotor middleArmJoint;
    private CRServo finalArmJoint;

    private Servo leftClawServo, rightClawServo;

    private boolean leftClawOpen = true;
    private boolean rightClawOpen = true;
    private Servo planeServo;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU controlHubIMU;


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
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        while (opModeIsActive()){}
    }
    private void positionToIntake(){}
    private void positionToScore(){}
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
        middleArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void assignHardwareToMotors() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        middleArmJoint = hardwareMap.get(DcMotor.class, "middleArmJoint");
        finalArmJoint = hardwareMap.get(CRServo.class,"finalArmJoint" );
        leftClawServo = hardwareMap.get(Servo.class,"leftClawServo" );
        rightClawServo = hardwareMap.get(Servo.class,"rightClawServo" );
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");
        elevatorRight = hardwareMap.dcMotor.get("elevatorRight");
        elevatorLeft = hardwareMap.dcMotor.get("elevatorLeft");

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
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        middleArmJoint.setDirection(DcMotor.Direction.REVERSE);
        finalArmJoint.setDirection(CRServo.Direction.FORWARD);
        elevatorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void turnOnRunToPosition() {
        setAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //*************************INITIALIZE*************************
}
