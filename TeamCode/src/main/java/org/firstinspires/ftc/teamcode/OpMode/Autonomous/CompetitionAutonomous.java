package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name= "competAuton")
public class CompetitionAutonomous extends LinearOpMode  {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private  DcMotor middleArmJoint;
    private CRServo finalArmJoint;
    private Servo leftClawServo, rightClawServo;
    private final int LEFT_CLAW_CLOSED = 0;
    private final int LEFT_CLAW_OPENED = 1;
    private final int RIGHT_CLAW_CLOSED = 1;
    private final int RIGHT_CLAW_OPENED = 0;

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private IMU imu;
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

    private final double COUNTS_PER_MOTOR_REV = 28;    //כמות צעדים עבור סיבוב מנוע
    private final double DRIVE_GEAR_REDUCTION = 20.0;     // יחס גירים
    private final double GOB_WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    private final double REV_WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    private final double GOB_WHEEL_BASE_DISTANCE = 39;     // For figuring circumference
    private final double REV_WHEEL_BASE_DISTANCE = 37;     // For figuring circumference
    private final double TURN_SPEED= 0.8;
    private final double MIN_TURN_SPEED = 0.2;
    private final double HEADING_THRESHOLD= 0.5 ;    // How close must the heading get to the target before moving to next step.
    private final double P_DRIVE_GAIN= 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    private final double P_TURN_GAIN= 0.03;     // Larger is more responsive, but also less stable
    private final int COLOR_THRESHOLD = 400; // blue
    //AprilTagProcessor tagProcessor;
    //VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();


        closeClaws();
        waitForStart();
        runAutonomous();
    }
    private void runAutonomous(){
        Side_Long_Gate_Middle(false);
    }



    //************************* - AUTONOMOUS MODES - *************************
    private void Side_Short_Corner(boolean blue){
        if(blue) {
            autonPutPixelInMiddle();
            turnToHeading(0.5, -85);
            //driveToColor(COLOR_THRESHOLD);
            forward(0.8, 87, 10000, true);

        }
        else{
            autonPutPixelInMiddle();
            turnToHeading(0.5, 85);
            forward(0.8, 82, 10000, true);
        }

    } //finished
     private void Side_Short_Middle(boolean blue){

        if(blue) {
            autonPutPixelInMiddle();
            right(0.8, 70, 30000, true);
            backwards(0.8, 100, 10000, true);
            right(0.8, 50, 10000, true);
        }
        else{
            autonPutPixelInMiddle();
            left(0.8, 70, 30000, true);
            backwards(0.8, 100, 10000, true);
            left(0.8, 50, 10000, true);
        }


    } // finished
    private void Side_Long_Straight_Corner(boolean blue){
        if(blue) {
            autonPutPixelInMiddle();
            backwards(0.5, 10, 2000, true);
            turnToHeading(1, -90);
            forward(0.5, 210, 10000, true);
            //checked!!!!!
        }
        else {
            autonPutPixelInMiddle();
            backwards(0.5, 7, 2000, true);
            turnToHeading(1, 90);
            forward(0.5, 210, 10000, true);
            //checked!!!!!
        }

    } // finished
    private void Side_Long_Straight_Middle(boolean blue){
        if(blue) {
            autonPutPixelInMiddle();
            backwards(0.8, 10, 2000, true);
            turnToHeading(1, -90);
            forward(1, 180, 10000, true);
            right(1,120,10000, true);
            forward(1,30,10000, true);
            //checked!!!!!
        }
        else {
            autonPutPixelInMiddle();
            backwards(0.8, 10, 2000, true);
            turnToHeading(1, 90);
            forward(1, 210, 10000, true);
            left(1,120,10000, true);
            forward(1,30,10000, true);
            //checked!!!!!
        }


        //driveToDistanceFromObject(1,20, 20000);
    }//finished
    private void Side_Long_Gate_Corner(boolean blue){
        if(blue) {
            autonPutPixelInMiddle();
            left(0.7, 58, 10000, true);
            backwards(0.5, 75, 10000, true);
            turnToHeading(0.7, 30);
            backwards(0.7, 60, 1000, true);
            turnToHeading(0.5, -90);
            forward(0.5,50,10,false ) ;
            forward(1, 125, 1000, true);
            left(0.7, 115, 2000, true);
            forward(1, 25, 10000, true);
            //checked!!!!!
        }
        else {
            right(0.7, 58, 10000, true);
            backwards(0.5, 75, 10000, true);
            turnToHeading(0.7, -30);
            backwards(0.7, 60, 1000, true);
            turnToHeading(0.7, 90);
            forward(0.5, 50, 10, false);
            forward(1, 125, 1000, true);
            right(0.7, 115, 10000, true);
            forward(1, 35, 10000, true);
            //checked!!!!!


        }
    }// finished

    private void Side_Long_Gate_Middle(boolean blue){
        if (blue) {
            autonPutPixelInMiddle();
            left(0.7, 63, 10000, true);
            backwards(0.7, 75, 10000, true);
            turnToHeading(0.7, 30);
            backwards(0.7, 50, 1000, true);
            turnToHeading(1, 90);
            backwards(1, 210, 10000, true);
        }
        else {
            autonPutPixelInMiddle();
            right(0.7, 63, 2000, true);
            backwards(0.7, 75, 10000, true);
            turnToHeading(0.7, -30);
            backwards(0.7, 50, 1000, true);
            turnToHeading(1, 90);
            forward(1, 220, 10000, true);

        } //checked!!!!!


    } // finished
    private void autonPutPixelInMiddle() {
        backwards(0.7, 68,10000 , true);
        forward(0.7, 60,10000 , false);
        forward(0.2, 10,400, true);
    }

    //************************* - AUTONOMOUS MODES - *************************

    private void closeClaws() {
        closeLeftClaw();
        closeRightClaw();
    }
    private void openLeftClaw(){
        leftClawServo.setPosition(LEFT_CLAW_OPENED);
    }
    private void closeLeftClaw(){
        leftClawServo.setPosition(LEFT_CLAW_CLOSED);
    }
    private void closeRightClaw(){
        rightClawServo.setPosition(RIGHT_CLAW_CLOSED);
    }


    private void goToAprilTag(AprilTagDetection tag){
        double x  = tag.ftcPose.x;
        double y  = tag.ftcPose.y;
        if(x>0){
            right(0.3, x, 10000, true);
        }
        if(x<0){
            left(0.3, -x, 10000, true);
        }
        double yaw = tag.ftcPose.yaw;
        turnToHeading(0.3,90);
        if(x>0){
            right(0.3, x, 10000, true);
        }
        if(x<0){
            left(0.3, -x, 10000, true);
        }
        forward(0.3,tag.ftcPose.y, 10000, true);

    }
    private void initCamera(){
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .build();
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .build();
    }







//************************* - DIRECTIONAL DRIVE - *************************
    private void forward(double speed, double distance, double timeOut, boolean stopOnEnd){
        distance *= DIST_NORM;
        driveDistanceByEncoder(speed, distance, distance,distance,distance, timeOut, stopOnEnd);
    }
    private void backwards(double speed, double distance, double timeOut, boolean stopOnEnd){
        distance *= DIST_NORM;
        driveDistanceByEncoder(speed, -distance, -distance,-distance,-distance, timeOut, stopOnEnd);
    }
    private void right(double speed, double distance, double timeOut, boolean stopOnEnd){
        distance *= SIDE_DIST_NORM;
        driveDistanceByEncoder(speed, distance, -distance, -distance,distance, timeOut, stopOnEnd);
    }
    private void left(double speed, double distance, double timeOut, boolean stopOnEnd){
        distance *= SIDE_DIST_NORM;
        driveDistanceByEncoder(speed, -distance, distance,distance,-distance, timeOut, stopOnEnd);
    }
    //************************* - DIRECTIONAL DRIVE - *************************



    //************************* - SUPERVISED DRIVE - *************************
    private void driveDistanceByEncoder(double speed,
                                        double leftFrontCm, double rightFrontCm, double leftRearCm, double rightRearCm,
                                        double timeoutS, boolean stopOnEnd) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            setDriveNewTargetPosition(leftFrontCm, rightFrontCm, leftRearCm, rightRearCm);
            turnOnRunToPosition();
            runtime.reset();

            while ((opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (matchRuntime.seconds() < 30000) &&
                    (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()))) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
                runToPosition(speed);
            }
            // Stop all motion;
            if(stopOnEnd) stopAllMotion();

            if(runtime.seconds() > timeoutS) {
                telemetry.addData("runtime", "******TIME OUT******");
                telemetry.update();
            }


            //sleep(sleepTime);   // optional pause after each move.

        }
    }
    private void moveRobot(double linearSpeed, double turnSpeed) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(linearSpeed+turnSpeed);
        leftRear.setPower(linearSpeed+turnSpeed);
        rightFront.setPower(linearSpeed-turnSpeed);
        rightRear.setPower(linearSpeed-turnSpeed);
    } // allows a free rotation and endless movement of the robot
    private void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN, maxTurnSpeed);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(targetHeading - getHeading()) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN, maxTurnSpeed);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            telemetry.addData("Heading", "%5.2f : %5.2f : %2.2f", targetHeading, getHeading(), turnSpeed);
            telemetry.update();
        }
        moveRobot(0, 0);
        telemetry.addData("Heading- error ", "%5.2f : %5.2f", getHeading(), targetHeading );
        telemetry.update();
        // Stop all motion;
        moveRobot(0, 0);
    } // turns to fixed angles on the field regardless of the robots orientation
    private void driveToDistanceFromObject(double speed, double distance, double timeoutS ){
        while ((opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (matchRuntime.seconds() < 30000) &&
                (distanceSensor.getDistance(DistanceUnit.CM) > distance))) {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
            setAllMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runToPosition(speed);
        }
        stopAllMotion();

    } // does not work on field borders
    private void driveToColor(double color){
        moveRobot(0.5, 0);
        while(!isBlue()){
            continue;
        }
        stopAllMotion();
    }
    //************************* - SUPERVISED DRIVE - *************************



    //************************* - INFORMATION AND CALCULATION - *************************
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private double getSteeringCorrection(double desiredHeading, double proportionalGain, double turnSpeed) {
        targetHeading = desiredHeading;  // Save for telemetry
        boolean rightTurn = true;

        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;

        // Determine the heading current error
        double heading = getHeading();

        if (heading >= targetHeading)  {
            headingError = heading - targetHeading;
            if (headingError > 180) {
                headingError = 360 - headingError;
                rightTurn = false;
            }
        }
        else {
            headingError = targetHeading - heading;
            if (headingError < 180) {
                rightTurn = false;
            }
            else {
                headingError = 360 -headingError;
            }
        }

        double speed =  calc_turn_speed(headingError, turnSpeed);
        if (rightTurn) return speed;
        else return -speed;
    }
    private double calc_turn_speed (double headingError, double speed) {

        if (Math.abs(headingError) > 10) return Math.min(speed, TURN_SPEED);
        return MIN_TURN_SPEED;
    }
    private boolean isBlue() {
        if ( colorSensor.blue() < COLOR_THRESHOLD) return true;
        return false;
    }
    //************************* - INFORMATION AND CALCULATION - *************************

    private void runToPosition(double speed) {

        runtime.reset();
        leftFront.setPower(Math.min(speed,driveMax));
        rightFront.setPower(Math.min(speed,driveMax));
        leftRear.setPower(Math.min(speed,driveMax));
        rightRear.setPower(Math.min(speed,driveMax));
    }
    private void stopAllMotion() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    private void setDriveNewTargetPosition(double leftFrontCm, double rightFrontCm, double leftRearCm, double rightRearCm) {

        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontCm * COUNTS_PER_CM);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontCm * COUNTS_PER_CM);
        newLeftRearTarget = leftRear.getCurrentPosition() + (int)(leftRearCm * COUNTS_PER_CM);
        newRightRearTarget = rightRear.getCurrentPosition() + (int)(rightRearCm * COUNTS_PER_CM);

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);
    }
    private void updateTelemetry(){

    }



    //************************* - INITIALIZE - *************************
    private void initializeRobot() {
        initRobotParameters();
        initMotors();
        initIMU();
        initCamera();


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
    private void initMotors(){
        assignHardwareToMotors();

        //intakeMotor = hardwareMap.dcMotor.get("intake");
        // liftMotor = hardwareMap.dcMotor.get("lift");

        initMotorsDirection();

        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        middleArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        imu.resetYaw();
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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ds-1");

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
        finalArmJoint.setDirection(CRServo.Direction.FORWARD);
    }
    private void turnOnRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //*************************INITIALIZE*************************
    //ariel is dead




}
