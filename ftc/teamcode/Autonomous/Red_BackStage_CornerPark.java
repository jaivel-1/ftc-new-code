package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.Pipelines.WebcamPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red Backstage CORNER PARK", group="Robot")
//@Disabled
// BSB CENTER PARK:
public class Red_BackStage_CornerPark extends LinearOpMode
{
    // List of Variables:
    long sleeptime = 100;
    double turnSpeed = 0.3;
    double driveSpeed = 0.5;
    double TestDistance = 10.0;
    static OpenCvCamera webcam1;
    public static WebcamPipeline detector=null;
    public static Prop location = Prop.RIGHT;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // MUST VERIFY
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    int PIXEL_RANDOMIZED=3;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        Path_Constants.SetData();
        MecanumRobotUtilities.InitializeHardware(Red_BackStage_CornerPark.this);
        ArmRobotUtilities.InitializeArm(Red_BackStage_CornerPark.this);
        ArmRobotUtilities.ResetArm();
        ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperClosedPosition);
        ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristUpPosition);

        InitializeCamera(Red_BackStage_CornerPark.this);
        SetPipeline();

        waitForStart();
        startCamera(Red_BackStage_CornerPark.this);

        while (opModeIsActive()) {
            if (location == Prop.CENTER) {
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.4,
                                Path_Constants.BSR_WP_Center[0], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Wrist Down:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristDownPosition);
                sleep(sleeptime);
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.4,
                                Path_Constants.BSR_WP_Center[1], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, driveSpeed,
                                Path_Constants.BSR_WP_Center[2], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Closed Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperClosedPosition);
                sleep(1000);
                // Wrist Up:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristUpPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, driveSpeed,
                                Path_Constants.BSR_WP_Center[3], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Turn Left:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.SPINANTICLOCKWISE, turnSpeed,
                                Path_Constants.BSR_WP_Center[4], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.6,
                                Path_Constants.BSR_WP_Center[5], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Raise Arm:
                ArmRobotUtilities.encoderArm(ArmRobotUtilities.armSpeed * 2,
                        ArmRobotUtilities.armScoreLeftPosition, 5.0,
                        Red_BackStage_CornerPark.this);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                // Drop Arm: (only slightly)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntermediate, 5.0,
                        Red_BackStage_CornerPark.this);
                // Strafe Left:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.LEFT, driveSpeed,
                                Path_Constants.BSR_WP_Center[8], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Drop Arm: (to final position)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntakePosition, 5.0,
                        Red_BackStage_CornerPark.this);

                // END OF AUTONOMOUS
            } else if (location == Prop.LEFT)
            {
                // Strafe Right:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.RIGHT, 0.35,
                                Path_Constants.BSR_WP_Left[0], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.35,
                                Path_Constants.BSR_WP_Left[1], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Spin Anticlockwise:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.SPINANTICLOCKWISE, 0.35,
                                Path_Constants.BSR_WP_Left[2], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Wrist Down:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristDownPosition);
                sleep(sleeptime);
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.35,
                                Path_Constants.BSR_WP_Left[3], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.35,
                                Path_Constants.BSR_WP_Left[4], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Close Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperClosedPosition);
                sleep(sleeptime);
                // Wrist Up:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristUpPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.35,
                                Path_Constants.BSR_WP_Left[5], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Strafe Right:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.RIGHT, 0.35,
                                Path_Constants.BSR_WP_Left[6], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Raise Arm:
                ArmRobotUtilities.encoderArm(ArmRobotUtilities.armSpeed * 2,
                        ArmRobotUtilities.armScoreLeftPosition, 5.0,
                        Red_BackStage_CornerPark.this);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                // Drop Arm: (only slightly)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntermediate, 5.0,
                        Red_BackStage_CornerPark.this);
                // Strafe Left:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.LEFT, 0.35,
                                Path_Constants.BSR_WP_Left[7], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Drop Arm: (to final position)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntakePosition, 5.0,
                        Red_BackStage_CornerPark.this);
                sleep(sleeptime);

                // END OF AUTONOMOUS

            }
           else {
                // Strafe Right:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.RIGHT, 0.35,
                                Path_Constants.BSR_WP_Right[0], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.35,
                                Path_Constants.BSR_WP_Right[1], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Wrist Down:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristDownPosition);
                sleep(sleeptime);
                // Move Forward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.FORWARD, 0.35,
                                Path_Constants.BSR_WP_Right[2], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.35,
                                Path_Constants.BSR_WP_Right[3], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Close Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperClosedPosition);
                sleep(sleeptime);
                // Wrist Up:
                ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristUpPosition);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.35,
                                Path_Constants.BSR_WP_Right[4], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Spin Anticlockwise:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.SPINANTICLOCKWISE, 0.35,
                                Path_Constants.BSR_WP_Right[5], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Move Backward:
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.BACKWARD, 0.35,
                                Path_Constants.BSR_WP_Right[6], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Raise Arm:
                ArmRobotUtilities.encoderArm(ArmRobotUtilities.armSpeed * 2,
                        ArmRobotUtilities.armScoreLeftPosition, 5.0,
                        Red_BackStage_CornerPark.this);
                // Open Gripper:
                ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
                // Drop Arm: (only slightly)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntermediate, 5.0,
                        Red_BackStage_CornerPark.this);
                // Strafe Left
                MecanumRobotUtilities.encoderDrive
                        (Auto_Struct.RobotDirection.LEFT, driveSpeed,
                                Path_Constants.BSR_WP_Right[7], 10.0,
                                Red_BackStage_CornerPark.this);
                sleep(sleeptime);
                // Drop Arm: (to final position)
                ArmRobotUtilities.encoderArm(-ArmRobotUtilities.armSpeed * 2.5,
                        ArmRobotUtilities.armIntakePosition, 5.0,
                        Red_BackStage_CornerPark.this);
                // END OF AUTONOMOUS:
            }
                terminateOpModeNow();

            }

        }

        // Initialize Camera

        public void InitializeCamera (LinearOpMode opMode)

    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        if (detector==null)
        {detector = new WebcamPipeline(telemetry, StartPosition.RED_STAGE);}
        telemetry.addData("Status", "Initialized Camera");
        telemetry.update();
    }


    // Set Pipeline
    public   void SetPipeline()
    {
        webcam1.setPipeline(detector);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.addData("Status", "Set Camera Pipeline");
        telemetry.update();
    }

    // Start Camera
    public void startCamera (LinearOpMode opMode)
    {
        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            opMode.telemetry.addData("ERROR", "Start was pressed too soon.");
            opMode.telemetry.update();

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
                totalTimeWaited += (webcam1.getOverheadTimeMs() * 4);
                opMode.sleep(webcam1.getOverheadTimeMs() * 4L);
            }
            opMode.telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                opMode.telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
            }
            opMode.telemetry.update();
        }
        else {
            opMode.telemetry.addData("INFO", "Pipeline is running correctly");
            opMode.telemetry.update();
        }
        location = Prop.RIGHT;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam1.stopStreaming();
            webcam1.closeCameraDevice();
        }

        opMode.telemetry.addData("Running path", detector.getPropLocation().toString(), location);
        opMode.telemetry.update();


    }

}
