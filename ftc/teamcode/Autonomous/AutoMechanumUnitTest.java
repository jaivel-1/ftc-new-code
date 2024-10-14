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

@Autonomous(name="Auto Mechanum Unit Test", group="Robot")
//@Disabled
public class AutoMechanumUnitTest extends LinearOpMode
{
    long sleeptime =3000;
    double TestDistance =50.0;
    static OpenCvCamera webcam1;
    public static WebcamPipeline detector=null;
    public static Prop location = Prop.RIGHT;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

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
    public void runOpMode()
    {
        // initialize  stuff
        MecanumRobotUtilities.InitializeHardware(AutoMechanumUnitTest.this);
        ArmRobotUtilities.InitializeArm(AutoMechanumUnitTest.this);
        ArmRobotUtilities.ResetArm();
        InitializeCamera(AutoMechanumUnitTest.this);
        SetPipeline();

        waitForStart();
        startCamera(AutoMechanumUnitTest.this);

        while (opModeIsActive())
        {
            telemetry.addData ("prop location",CameraRobotUtilities.location.toString());
            telemetry.update();
            /*
            ArmRobotUtilities.encoderArm(0.3,ArmRobotUtilities.armScoreLeftPosition,5.0,AutoMechanumUnitTest.this);
            sleep(sleeptime);

            ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperOpenPosition);
            sleep(sleeptime);

            ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristDownPosition);
            sleep(sleeptime);

            ArmRobotUtilities.OperateGripper(ArmRobotUtilities.gripperClosedPosition);
            sleep(sleeptime);

            ArmRobotUtilities.OperateWrist(ArmRobotUtilities.wristUpPosition);
            sleep(sleeptime);

*/
           /* MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.FORWARD,0.1,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);

            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.BACKWARD,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime); */
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.RIGHT,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);

            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.LEFT,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
/*
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.SPINCLOCKWISE,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.SPINANTICLOCKWISE,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);



            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.LEFTFRONTDIAGONAL,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.RIGHTFRONTDIAGONAL,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);

            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.LEFTBACKWARDDIAGONAL,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.RIGHTBACKWARDDIAGONAL,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);

            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.ROTATELEFTBACKPIVOT_ANTICLOCK,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.ROTATERIGHTBACKPIVOT_CLOCK,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            sleep(sleeptime);
            MecanumRobotUtilities.encoderDrive(Auto_Struct.RobotDirection.ROTATEBACKCENTERPIVOT_ANTICLOCK,0.3,TestDistance,5.0, AutoMechanumUnitTest.this);
            */

            //terminateOpModeNow();

        }

    }
    // initialize Camera

    public  void InitializeCamera(LinearOpMode opMode )
    {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        if (detector==null)
        {detector = new WebcamPipeline(opMode.telemetry, StartPosition.BLUE_STAGE);}
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
