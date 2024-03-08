package org.firstinspires.ftc.teamcode.OpMode.Autonomous;



import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="GuyTeleOp", group="Robot")


public class GuyAuton extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;    //כמות צעדים עבור סיבוב מנוע
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // יחס גירים
    static final double GOB_WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double REV_WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    static final double GOB_WHEEL_BASE_DISTANCE = 39;     // For figuring circumference
    static final double REV_WHEEL_BASE_DISTANCE = 37;     // For figuring circumference

    private Rev2mDistanceSensor sensorTimeOfFlight;
    private DistanceSensor sensorDistance;
    static boolean MAIN_ROBOT = false; // false for ROBOT_B
    private double DIST_NORM;
    private static double WHEEL_DIAMETER_CM;
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.4;
    static final double MIN_TURN_SPEED = 0.02;
    double WHEEL_BASE_DISTANCE;
    double FULL_ROUND;
    static int newLeftFrontTarget;
    static int newRightFrontTarget;
    static int newLeftRearTarget;
    static int newRightRearTarget;
    double COUNTS_PER_CM;
    IMU.Parameters myIMUparameters;
    int sleepTime = 30;
    IMU imu;
    private double targetHeading = 0;
    private double headingError = 0;
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.04;     // Larger is more responsive, but also less stable

    private double driveSpeed = 0.6;
    private double turnSpeed = 0.6;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;




    @Override
    public void runOpMode() {
        //initRobot();
        //initIMU();
        // Initialize the drive system variables.
        //initializeMotors();
        //sendSuccessfulMotorResetMessage();
        /*
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
        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        while(opModeIsActive()){
            if(tagProcessor.getDetections().size()>0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x",tag.ftcPose.x);
                telemetry.addData("y",tag.ftcPose.y);
                telemetry.addData("z",tag.ftcPose.z);
                telemetry.addData("roll",tag.ftcPose.roll);
                telemetry.addData("pitch",tag.ftcPose.pitch);
                telemetry.addData("yaw",tag.ftcPose.yaw);
                telemetry.addData("id",tag.id);
            }

            telemetry.update();
            if (gamepad1.a){
                turnToTag(4);
            }

        }

         */
        initTfod();

        waitForStart();

        while(opModeIsActive()){
            telemetryTfod();

        }
    }
    TfodProcessor tfod;

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (true) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private void turnToTag(int targetTagId){
        AprilTagDetection tag;
        AprilTagDetection tag1;
        AprilTagDetection tag2;
        AprilTagDetection tag3;


            /*if (tagProcessor.getDetections().size() > 0) {
                tag1 = tagProcessor.getDetections().get(0);
                if (tagProcessor.getDetections().size() > 1) {
                    tag2 = tagProcessor.getDetections().get(1);
                    if (tagProcessor.getDetections().size() > 2) {
                        tag3 = tagProcessor.getDetections().get(2);
                    }
                }
            }

             */

            if(getTagById(targetTagId,tagProcessor.getDetections() )!=null){
                telemetry.addData("tagid",getTagById(targetTagId,tagProcessor.getDetections()).id );

                while(getTagById(targetTagId,tagProcessor.getDetections())!=null){
                    tag = getTagById(targetTagId,tagProcessor.getDetections());

                    //if x positive, turn right
                    if(tag.ftcPose.x>0){
                        rotateRight();
                    }

                    //if x negative, turn left
                    if(tag.ftcPose.x<0){
                        rotateLeft();
                    }
                    if(tag.ftcPose.x == 0){
                        stopAllMotion();
                    }

                }


            }




    }
    private void rotateLeft(){
        leftRear.setPower(-0.2);
        leftFront.setPower(-0.2);
        rightRear.setPower(0.2);
        rightFront.setPower(0.2);

    }
    private void rotateRight(){
        leftRear.setPower(0.2);
        leftFront.setPower(0.2);
        rightRear.setPower(-0.2);
        rightFront.setPower(-0.2);


    }

    private AprilTagDetection getTagById(int id, ArrayList<AprilTagDetection> detections){
        for (int i = 0; i < detections.size(); i++) {
            if(detections.get(i).id== id){
                return detections.get(i);
            }

        }
        return null;
    }



    private void initRobot() {
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ));
        if (MAIN_ROBOT) {
            WHEEL_DIAMETER_CM = GOB_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = GOB_WHEEL_BASE_DISTANCE;
            DIST_NORM = 1;

        } else {
            //sensorDistance = hardwareMap.get(DistanceSensor.class, "ds-1");

            WHEEL_DIAMETER_CM = REV_WHEEL_DIAMETER_CM;
            WHEEL_BASE_DISTANCE = REV_WHEEL_BASE_DISTANCE;
            DIST_NORM = 0.9;

        }
        COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_CM * 3.1415);
        FULL_ROUND = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * (WHEEL_BASE_DISTANCE / WHEEL_DIAMETER_CM) * 1.65;
    }

    private void initIMU() {


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot

// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(myIMUparameters);
        imu.resetYaw();


        /*imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUparameters);

        imu.resetYaw();

         */
    }

    private void initializeMotors() {
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

    private void sendSuccessfulMotorResetMessage() {
        telemetry.addData("Starting at", "%7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void forward(double speed, double distance, double timeOut){
        encoderDrive(speed, distance, distance,distance,distance, timeOut);
    }
    private void right(double speed, double distance, double timeOut){
        encoderDrive(speed, distance, -distance, -distance,distance, timeOut);
    }
    private void backwards(double speed, double distance, double timeOut){
        encoderDrive(speed, -distance, -distance,-distance,-distance, timeOut);
    }
    private void left(double speed, double distance, double timeOut){
        encoderDrive(speed, -distance, distance,distance,-distance, timeOut);
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

    private void runToPosition(double speed) {
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));
    }

    private void encoderDrive(double speed,
                              double leftFrontCm, double rightFrontCm, double leftRearCm, double rightRearCm,
                              double timeoutS) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            setDriveNewTargetPosition(leftFrontCm, rightFrontCm, leftRearCm, rightRearCm);

            turnOnRunToPosition();

            runToPosition(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()))) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            stopAllMotion();
            if(runtime.seconds() > timeoutS){
                telemetry.addData("runtime", "******TIME OUT******");
                telemetry.update();
                sleep(sleepTime);   // optional pause after each move.
            }

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(sleepTime);   // optional pause after each move.
        }
    }


    private void stopAllMotion() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(sleepTime);   // optional pause after each move.
    }

    private void turnOnRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}