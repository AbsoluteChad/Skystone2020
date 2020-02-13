package org.firstinspires.ftc.teamcode.lib.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector {

    //Values of the 3 blocks (skystone or not)
    private static int leftVal = -1;
    private static int middleVal = -1;
    private static int rightVal = -1;

    //Dimensions of rectangles on screen
    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float xOffset = -1f/8f;//-4f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float yOffset = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    //Position of rectangles on screen
    private static float[] leftRectPos = {2f/8f + xOffset, 4f/8f + yOffset}; //0 = col, 1 = row
    private static float[] middleRectPos = {4f/8f + xOffset, 4f/8f + yOffset};
    private static float[] rightRectPos = {5.5f/8f + xOffset, 4f/8f + yOffset};

    //Screen dimensions
    private final int rows = 640;
    private final int cols = 480;

    //Declare camera
    private OpenCvCamera phoneCamera;

    public SkystoneDetector(HardwareMap hardwareMap) {
        //Init phone camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Start pipeline
        openCamera();
        setPipeline(new SkystonePipeline());
        startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    public SkystoneDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        //Init phone camera
        telemetry.addData("Status", "SkystoneDetector");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        telemetry.addData("Status", "Syke" + cameraMonitorViewId);
        telemetry.update();

        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        telemetry.addData("Status", "ViewId");
        telemetry.update();

        //Start pipeline
        openCamera();
        setPipeline(new SkystonePipeline());
        startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    /**
     * Gets the position of the skystone (assuming phone camera sees 3 blocks in which one is a skystone)
     * @return the skystone position as a char 'L', 'M', or 'R' for "Left" "Middle" and "Right". Will return
     * 'N' if the skystone cannot properly be sensed.
     */
    public char getSkystonePosition() {
        char skystonePosition = 'N';
        if (leftVal == 0) {
            skystonePosition = 'L';
        } else if (middleVal == 0) {
            skystonePosition = 'M';
        } else if (rightVal == 0) {
            skystonePosition = 'R';
        }
        return skystonePosition;
    }

    /**
     * Opens camera device
     */
    public void openCamera() {
        phoneCamera.openCameraDevice();
    }

    /**
     * Closes camera device
     */
    public void closeCamera() {
        phoneCamera.closeCameraDevice();
    }

    /**
     * Sets pipeline for vision
     * @param pipeline desired vision pipeline
     */
    public void setPipeline(OpenCvPipeline pipeline) {
        phoneCamera.setPipeline(pipeline);
    }

    /**
     * Starts viewport to Driver Station phone
     * @param camRotation rotation of camera mounted on robot
     */
    public void startStreaming(int rows, int cols, OpenCvCameraRotation camRotation) {
        phoneCamera.startStreaming(rows, cols, camRotation);
    }
    
    /**
     * Stops viewport to Driver Station phone
     */
    public void stopStreaming() {
        phoneCamera.stopStreaming();
    }

    /**
     * Pauses viewport to driver station phone
     * NOTE: pipeline does not stop
     */
    public void pauseViewport() {
        phoneCamera.pauseViewport();
    }

    /**
     * Resumes viewport to driver station phone (if paused)
     */
    public void resumeViewport() {
        phoneCamera.resumeViewport();
    }

    /**
     * @return the camera fps
     */
    public double getFPS() {
        return phoneCamera.getFps();
    }

    /**
     * Pipeline class used for detecting skystone
     */
    private static class SkystonePipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {
            DETECTION, //Includes outlines
            THRESHOLD //Black & white
        }

        private Stage viewportRenderStage = Stage.DETECTION;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = viewportRenderStage.ordinal();
            int nextStageNum = currentStageNum + 1;
            if(nextStageNum >= stages.length) {
                nextStageNum = 0;
            }
            viewportRenderStage = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            //Erase existing contours
            contoursList.clear();

            /*
             * Determines Cb color difference
             * Lower Cb -- more blue = white --> skystone
             * Higher Cb -- less blue = grey --> regular stone
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//Converts RGB to YCbCr
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//Finds Cb difference and stores

            //Black & white threshold
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //Outline/Contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8); //Draw blue contours

            //Get values from frame at circle
            double[] leftPixel = thresholdMat.get((int) (input.rows() * leftRectPos[1]), (int) (input.cols() * leftRectPos[0]));
            leftVal = (int) leftPixel[0];

            double[] middlePixel = thresholdMat.get((int) (input.rows() * middleRectPos[1]), (int) (input.cols() * middleRectPos[0]));
            middleVal = (int) middlePixel[0];

            double[] rightPixel = thresholdMat.get((int) (input.rows() * rightRectPos[1]), (int) (input.cols() * rightRectPos[0]));
            rightVal = (int) rightPixel[0];

            //Create three points
            Point pointLeft = new Point((int)(input.cols()* leftRectPos[0]), (int)(input.rows()* leftRectPos[1]));
            Point pointMiddle = new Point((int)(input.cols()* middleRectPos[0]), (int)(input.rows()* middleRectPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightRectPos[0]), (int)(input.rows()* rightRectPos[1]));

            //Draw circles on those points
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);
            Imgproc.circle(all, pointMiddle, 5, new Scalar(255, 0, 0), 1);
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);

            //Draw rectangle 1 (1-3)
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(leftRectPos[0]-rectWidth/2),
                            input.rows()*(leftRectPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftRectPos[0]+rectWidth/2),
                            input.rows()*(leftRectPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3
            );

            //Draw rectangle 2 (3-5)
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(middleRectPos[0]-rectWidth/2),
                            input.rows()*(middleRectPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(middleRectPos[0]+rectWidth/2),
                            input.rows()*(middleRectPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3
            );

            //Draw rectangle 3 (5-7)
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(rightRectPos[0]-rectWidth/2),
                            input.rows()*(rightRectPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightRectPos[0]+rectWidth/2),
                            input.rows()*(rightRectPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3
            );

            switch (viewportRenderStage) {
                case THRESHOLD: return thresholdMat;
                case DETECTION: return all;
                default: return input;
            }
        }
    }
}
