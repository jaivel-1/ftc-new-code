package org.firstinspires.ftc.teamcode.Autonomous;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.Pipelines.Prop;
        import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
        import org.firstinspires.ftc.teamcode.Pipelines.WebcamPipeline;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
The program attempts to recognize a Team Prop on a spike mark, place the purple pixel on that mark
        and drop yellow pixel in the corresponding area on the backdrop.


    Starting position - blue alliance, back stage

    Program flow:
    - Use Pixel counting to detect which spike mark has the Team Prop
    - drive forward toward indicated spike mark,
    - Drop purple pixel (on spike mark) and yellow pixel on the ground
    - Drive backward and pick up yellow pixel
    - Drive backwards towards back drop
    - Drop yellow pixel in corresponding area
    - Park in zone right or left
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name="Camera Checker", group="Robot")
@Disabled
public class CameraCheck extends LinearOpMode
{

    OpenCvCamera webcam;
    /* Declare OpMode members. */


    OpenCvCamera camera;
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

    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.BLUE_STAGE);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
        }
        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " BLUE_STAGE_" + location);
        telemetry.update();
        while (opModeIsActive())
        {
            if (location==Prop.CENTER) {

                telemetry.addData("running", "Center");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            else if (location==Prop.LEFT)
            {  telemetry.addData("running", "Left");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            else
            {
                telemetry.addData("running", "Right");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            terminateOpModeNow();


        }
    }


}