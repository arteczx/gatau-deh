package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import java.util.*;

/**
 * class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    //Astrobee specifications
    private static final double MAX_VELOCITY = 0.5;  // m/s
    private static final double MIN_DISTANCE = 0.05; // m
    private static final double MIN_ANGLE = 7.5;     // degrees
    private static final double MAX_THRUST_X = 0.6;  // N
    private static final double ROBOT_MASS = 10.0;   // kg

    //KIZ boundaries
    private static final double[] KIZ1_BOUNDS = {10.3, -10.2, 4.32, 11.55, -6.0, 5.57};
    private static final double[] KIZ2_BOUNDS = {9.5, -10.5, 4.02, 10.5, -9.6, 4.8};

    //KOZ boundaries (x_min, y_min, z_min, x_max, y_max, z_max)
    private static final double[][] KOZ_BOUNDS = {
        //KOZ1
        {10.87, -9.5, 4.27, 11.6, -9.45, 4.97},    //pos 1
        {10.25, -9.5, 4.97, 10.87, -9.45, 5.62},   //pos 2
        //KOZ2
        {10.87, -8.5, 4.97, 11.6, -8.45, 5.62},    //pos 1
        {10.25, -8.5, 4.27, 10.7, -8.45, 4.97},    //pos 2
        //KOZ3
        {10.87, -7.40, 4.27, 11.6, -7.35, 4.97},   //pos 1
        {10.25, -7.40, 4.97, 10.87, -7.35, 5.62}   //pos 2
    };

    //safe waypoints for navigation
    private final Point[] safeWaypoints = {
        new Point(10.0, -9.8, 4.5),   //near dock
        new Point(10.3, -9.0, 4.5),   //between KOZ1 and KOZ2
        new Point(10.3, -8.0, 4.5),   //between KOZ2 and KOZ3
        new Point(10.3, -7.0, 4.5)    //past KOZ3
    };

    //def the coordinates for each area
    private final Point[] areaPoints = new Point[] {
        new Point(10.95, -10.585, 4.82), //area 1 center
        new Point(10.925, -8.875, 3.76203), //area 2 center
        new Point(10.925, -7.875, 3.76203), //area 3 center
        new Point(9.866984, -6.8525, 4.32)  //area 4 center
    };
    
    private final Point startPoint = new Point(9.815, -9.806, 4.293);
    private final Point astronautPoint = new Point(11.143, -6.7607, 4.9654);
    private final Quaternion startQuaternion = new Quaternion(1f, 0f, 0f, 0f);
    private final Quaternion astronautQuaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
    
    //map to store found items and their locations
    private HashMap<Integer, String> foundItemsMap = new HashMap<>();
    private int targetAreaId = -1;

    //define mapping between AR markers and items
    private final HashMap<Integer, String> markerToItem = new HashMap<Integer, String>() {{
        put(1, "tape");
        put(2, "top");
        put(3, "screwdriver");
        put(4, "beaker");
        put(5, "hammer");
        put(6, "pipette");
        put(7, "wrench");
        put(8, "thermometer");
        put(9, "watch");
        put(10, "goggle");
    }};

    //camera offsets from center point
    private static final double[] NAV_CAM_OFFSET = {0.1177, -0.0422, -0.0826};
    private static final double[] HAZ_CAM_OFFSET = {0.1328, 0.0362, -0.0826};

    //mt to get corrected point for NavCam view
    private Point getCorrectedPointForNavCam(Point target) {
        return new Point(
            target.getX() - NAV_CAM_OFFSET[0],
            target.getY() - NAV_CAM_OFFSET[1],
            target.getZ() - NAV_CAM_OFFSET[2]
        );
    }

    //mt to process and save debug images
    private void saveDebugImage(Mat image, String prefix, int counter) {
        try {
            String filename = String.format("%s_%d.png", prefix, counter);
            saveMatImage(image, filename);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //AR marker detection with debug
    private List<Integer> detectArUcoMarkers(Mat image, String debugPrefix) {
        List<Integer> detectedMarkers = new ArrayList<>();
        try {
            Mat gray = new Mat();
            Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
            
            //get NavCam intrinsics
            double[][] camParams = api.getNavCamIntrinsics();
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);
            
            //set camera matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    cameraMatrix.put(i, j, camParams[0][i * 3 + j]);
                }
            }
            //set distortion coefficients
            for (int i = 0; i < 5; i++) {
                distCoeffs.put(0, i, camParams[1][i]);
            }

            //enhance image
            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
            Imgproc.adaptiveThreshold(gray, gray, 255,
                    Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                    Imgproc.THRESH_BINARY, 11, 2);

            saveDebugImage(gray, debugPrefix + "_processed", detectedMarkers.size());
            
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            Mat ids = new Mat();
            
            //detect param
            DetectorParameters parameters = DetectorParameters.create();
            parameters.setAdaptiveThreshWinSizeMin(3);
            parameters.setAdaptiveThreshWinSizeMax(23);
            parameters.setAdaptiveThreshWinSizeStep(10);
            parameters.setMinMarkerPerimeterRate(0.03);
            parameters.setMaxMarkerPerimeterRate(0.5);
            
            Aruco.detectMarkers(gray, dictionary, corners, ids, parameters, new Mat(), cameraMatrix, distCoeffs);
            
            if (!ids.empty()) {
                // Draw markers for debug image
                Mat debugImage = image.clone();
                Aruco.drawDetectedMarkers(debugImage, corners, ids);
                saveDebugImage(debugImage, debugPrefix + "_detected", detectedMarkers.size());
                
                for (int i = 0; i < ids.rows(); i++) {
                    detectedMarkers.add((int) ids.get(i, 0)[0]);
                }
            }

            //clear
            gray.release();
            cameraMatrix.release();
            distCoeffs.release();
            ids.release();
            for (Mat corner : corners) {
                corner.release();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return detectedMarkers;
    }

    //mt to identify items based on AR markers
    private String identifyItem(List<Integer> markers) {
        if (!markers.isEmpty()) {
            Integer markerId = markers.get(0);
            return markerToItem.getOrDefault(markerId, "unknown");
        }
        return "unknown";
    }

    @Override
    protected void runPlan1() {
        api.startMission();

        //init path planning
        List<Point> currentPath = new ArrayList<>();
        currentPath.add(startPoint);

        //1.move through safe waypoints to start position
        for (Point waypoint : safeWaypoints) {
            if (isPointSafe(waypoint)) {
                moveToWithRetry(waypoint, startQuaternion);
            }
        }

        //2.Patrol each area using safe paths
        for (int i = 0; i < areaPoints.length; i++) {
            Point targetPoint = areaPoints[i];
            
            //check if direct path is safe, otherwise use waypoints
            if (!isPathSafe(api.getRobotKinematics().getPosition(), targetPoint)) {
                Point safePoint = findNearestSafeWaypoint(targetPoint);
                moveToWithRetry(safePoint, startQuaternion);
            }
            
            //move to scanning position if it's safe
            if (isPointSafe(targetPoint)) {
                Quaternion scanQuaternion = createScanningQuaternion(targetPoint);
                moveToWithRetry(targetPoint, scanQuaternion);
                
                scanArea(i, targetPoint, scanQuaternion);
            }
        }

        //3.move to astronaut position
        Point safeAstronautApproach = findSafeApproachPoint(astronautPoint);
        moveToWithRetry(safeAstronautApproach, astronautQuaternion);
        moveToWithRetry(astronautPoint, astronautQuaternion);
        
        api.reportRoundingCompletion();

        completeTargetOperation();
    }

    //helper mt for safety and navigation (KOZ & KIZ)

    private boolean isPointSafe(Point p) {
        boolean inKiz = isPointInBounds(p, KIZ1_BOUNDS) || isPointInBounds(p, KIZ2_BOUNDS);
        if (!inKiz) return false;

        for (double[] koz : KOZ_BOUNDS) {
            if (isPointInBounds(p, koz)) return false;
        }
        return true;
    }

    private boolean isPointInBounds(Point p, double[] bounds) {
        return p.getX() >= bounds[0] && p.getX() <= bounds[3] &&
               p.getY() >= bounds[1] && p.getY() <= bounds[4] &&
               p.getZ() >= bounds[2] && p.getZ() <= bounds[5];
    }

    private boolean isPathSafe(Point start, Point end) {
        if (!isValidDistance(start, end)) {
            return false;
        }

        //calculate required acceleration
        double distance = distance(start, end);
        double timeRequired = distance / MAX_VELOCITY;
        double acceleration = (2 * distance) / (timeRequired * timeRequired);
        double force = ROBOT_MASS * acceleration;
        
        if (force > MAX_THRUST_X) {
            return false;
        }

        //check path points
        int steps = 10;
        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;
            Point p = new Point(
                start.getX() + t * (end.getX() - start.getX()),
                start.getY() + t * (end.getY() - start.getY()),
                start.getZ() + t * (end.getZ() - start.getZ())
            );
            if (!isPointSafe(p)) return false;
        }
        return true;
    }

    private Point findNearestSafeWaypoint(Point target) {
        Point nearest = safeWaypoints[0];
        double minDist = Double.MAX_VALUE;
        
        for (Point waypoint : safeWaypoints) {
            if (isPointSafe(waypoint)) {
                double dist = distance(waypoint, target);
                if (dist < minDist) {
                    minDist = dist;
                    nearest = waypoint;
                }
            }
        }
        return nearest;
    }

    private double distance(Point p1, Point p2) {
        double dx = p1.getX() - p2.getX();
        double dy = p1.getY() - p2.getY();
        double dz = p1.getZ() - p2.getZ();
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    private Point findSafeApproachPoint(Point target) {
        //find a safe point slightly before the target
        double offset = 0.3; //30cm offset
        Point approach = new Point(
            target.getX() - offset,
            target.getY(),
            target.getZ()
        );
        return isPointSafe(approach) ? approach : findNearestSafeWaypoint(target);
    }

    //enhanced scanning with relative movement
    private void scanArea(int areaIndex, Point targetPoint, Quaternion baseQuaternion) {
        boolean itemFound = false;
        float[] angles = {0f, -15f, 15f, -30f, 30f};
        Point correctedPoint = getCorrectedPointForNavCam(targetPoint);
        
        for (float angle : angles) {
            if (itemFound) break;
            
            Quaternion rotatedQuat = adjustQuaternionByDegrees(baseQuaternion, angle);
            
            if (isPointSafe(targetPoint) && 
                isValidRotation(api.getRobotKinematics().getOrientation(), rotatedQuat)) {
                
                //try relative movement first
                Point relativePoint = new Point(0.1, 0, 0); //small forward movement
                api.relativeMoveTo(relativePoint, rotatedQuat, true);
                
                //wait for stabilization
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                for (int attempt = 0; attempt < 3 && !itemFound; attempt++) {
                    Mat image = api.getMatNavCam();
                    List<Integer> markers = detectArUcoMarkers(image, 
                        String.format("area_%d_angle_%d_attempt_%d", areaIndex, (int)angle, attempt));
                    
                    if (!markers.isEmpty()) {
                        String detectedItem = identifyItem(markers);
                        if (!"unknown".equals(detectedItem)) {
                            api.setAreaInfo(areaIndex + 1, detectedItem, 1);
                            foundItemsMap.put(areaIndex + 1, detectedItem);
                            itemFound = true;
                            
                            //save successful detection image
                            saveDebugImage(image, String.format("success_area_%d_%s", areaIndex, detectedItem), 0);
                            break;
                        }
                    }
                }
            }
        }
    }

    private void completeTargetOperation() {
        String targetItem = "unknown";
        Point correctedAstronautPoint = getCorrectedPointForNavCam(astronautPoint);
        
        //move to position for astronaut QR reading
        moveToWithRetry(correctedAstronautPoint, astronautQuaternion);
        
        for (int attempt = 0; attempt < 5 && "unknown".equals(targetItem); attempt++) {
            Mat astronautImage = api.getMatNavCam();
            List<Integer> targetMarkers = detectArUcoMarkers(astronautImage, 
                String.format("astronaut_target_attempt_%d", attempt));
            targetItem = identifyItem(targetMarkers);
            
            if (!"unknown".equals(targetItem)) {
                saveDebugImage(astronautImage, "target_item_detected", attempt);
            }
        }
        
        //notify target recognition
        api.notifyRecognitionItem();

        if (targetAreaId != -1) {
            Point targetPoint = areaPoints[targetAreaId - 1];
            Point correctedTargetPoint = getCorrectedPointForNavCam(targetPoint);
            
            if (isPointSafe(targetPoint)) {
                Quaternion targetQuaternion = createScanningQuaternion(targetPoint);
                moveToWithRetry(correctedTargetPoint, targetQuaternion);
                
                //take final snapshot
                Mat finalImage = api.getMatNavCam();
                saveDebugImage(finalImage, "final_target", 0);
                api.takeTargetItemSnapshot();
            }
        }
    }

    //helper mt to create scanning quaternion based on position
    private Quaternion createScanningQuaternion(Point target) {
        //calc direction to face the target
        float yaw = (float) Math.atan2(target.getY() - startPoint.getY(), 
                                     target.getX() - startPoint.getX());
        return new Quaternion(0f, 0f, (float) Math.sin(yaw/2), (float) Math.cos(yaw/2));
    }

    //helper mt to adjust quaternion by degrees
    private Quaternion adjustQuaternionByDegrees(Quaternion base, float degrees) {
        float radians = (float) Math.toRadians(degrees);
        float sin = (float) Math.sin(radians/2);
        float cos = (float) Math.cos(radians/2);
        return new Quaternion(
            base.getX()*cos - base.getY()*sin,
            base.getX()*sin + base.getY()*cos,
            base.getZ(),
            base.getW()
        );
    }

    //helper mtto handle movement with retry logic
    private void moveToWithRetry(Point point, Quaternion quaternion) {
        int maxRetries = 3;
        int retryCount = 0;
        boolean success = false;
        
        while (!success && retryCount < maxRetries) {
            try {
                api.moveTo(point, quaternion, false);
                success = true;
            } catch (Exception e) {
                retryCount++;
                if (retryCount >= maxRetries) {
                    throw e;
                }
            }
        }
    }

    //helper mt to check if movement distance is valid
    private boolean isValidDistance(Point p1, Point p2) {
        double dist = distance(p1, p2);
        return dist >= MIN_DISTANCE;
    }

    //helper mt to check if rotation angle is valid
    private boolean isValidRotation(Quaternion q1, Quaternion q2) {
        //convert quaternion difference to angle
        double dotProduct = q1.getX()*q2.getX() + q1.getY()*q2.getY() + 
                          q1.getZ()*q2.getZ() + q1.getW()*q2.getW();
        double angle = Math.toDegrees(2 * Math.acos(Math.abs(dotProduct)));
        return angle >= MIN_ANGLE;
    }

    //moveToWithRetry with speed control and minimum movement checks
    private void moveToWithRetry(Point targetPoint, Quaternion targetQuaternion) {
        int maxRetries = 3;
        int retryCount = 0;
        boolean success = false;
        
        Point currentPosition = api.getRobotKinematics().getPosition();
        Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();

        //check if movement is necessary
        if (!isValidDistance(currentPosition, targetPoint) && 
            !isValidRotation(currentQuaternion, targetQuaternion)) {
            return; //mvement too small, skip
        }

        while (!success && retryCount < maxRetries) {
            try {
                //calc movement time based on distance and max velocity
                double distance = distance(currentPosition, targetPoint);
                double moveTime = distance / MAX_VELOCITY;

                //movement param
                api.moveTo(targetPoint, targetQuaternion, true); //true for speed control
                success = true;

                //wait for movement to complete / obstacle detection
                int timeoutMs = (int)(moveTime * 1000) + 1000; //1 sec buffer
                api.waitForMotion(timeoutMs);

            } catch (Exception e) {
                retryCount++;
                if (retryCount >= maxRetries) {
                    throw e;
                }
                //wait before retry
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    @Override
    protected void runPlan2() {
        // Not used in preliminary round
    }

    @Override
    protected void runPlan3() {
        // Not used in preliminary round
    }
}
