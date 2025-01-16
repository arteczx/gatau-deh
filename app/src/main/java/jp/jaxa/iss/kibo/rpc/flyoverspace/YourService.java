package jp.jaxa.iss.kibo.rpc.flyoverspace;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.aruco.DetectorParameters;
import java.util.*;
import java.util.concurrent.*;

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

    //camera offsets and parameters
    private static final double[] NAV_CAM_OFFSET = {0.1177, -0.0422, -0.0826};
    private static final double[] HAZ_CAM_OFFSET = {0.1328, 0.0362, -0.0826};
    private Mat cameraMatrix;
    private Mat distCoeffs;

    //cache for optimization
    private final Map<String, Mat> imageCache = new ConcurrentHashMap<>();
    private final Map<Integer, Long> lastDetectionTime = new ConcurrentHashMap<>();
    private static final long DETECTION_COOLDOWN = 500; // ms

    //thread management
    private final ExecutorService executor = Executors.newFixedThreadPool(2);
    private final CompletableFuture<Void> imageProcessingFuture = new CompletableFuture<>();

    private static class Node implements Comparable<Node> {
        Point point;
        Node parent;
        double gCost; //cost from start to current node
        double hCost; //estimated cost from current node to goal
        
        public Node(Point point, Node parent, double gCost, double hCost) {
            this.point = point;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
        }
        
        public double getFCost() {
            return gCost + hCost;
        }
        
        @Override
        public int compareTo(Node other) {
            return Double.compare(this.getFCost(), other.getFCost());
        }
        
        @Override
        public boolean equals(Object obj) {
            if (obj instanceof Node) {
                Node other = (Node) obj;
                return point.getX() == other.point.getX() &&
                       point.getY() == other.point.getY() &&
                       point.getZ() == other.point.getZ();
            }
            return false;
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(point.getX(), point.getY(), point.getZ());
        }
    }

    @Override
    protected void runPlan1() {
        try {
            initializeCameraParameters();
            executeMainMission();
        } catch (Exception e) {
            e.printStackTrace();
            handleMissionFailure();
        } finally {
            cleanup();
        }
    }

    private void initializeCameraParameters() {
        double[][] camParams = api.getNavCamIntrinsics();
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        distCoeffs = new Mat(1, 5, CvType.CV_64F);
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.put(i, j, camParams[0][i * 3 + j]);
            }
        }
        for (int i = 0; i < 5; i++) {
            distCoeffs.put(0, i, camParams[1][i]);
        }
    }

    private void executeMainMission() {
        try {
            api.startMission();

            //1 thread: mov & nav
            CompletableFuture<Void> navigationFuture = CompletableFuture.runAsync(() -> {
                try {
                    moveToWithRetry(startPoint, startQuaternion);
                } catch (Exception e) {
                    handleEmergencyStop();
                    throw e;
                }
            }, executor);

            //2 thread: img proc prep
            CompletableFuture<Void> detectionFuture = CompletableFuture.runAsync(() -> {
                initializeCameraParameters();
            }, executor);

            try {
                navigationFuture.get(30, TimeUnit.SECONDS);
                detectionFuture.get(10, TimeUnit.SECONDS);
            } catch (TimeoutException e) {
                handleEmergencyStop();
                throw new RuntimeException("Mission initialization timeout", e);
            }

            // Scan setiap area menggunakan scanAreaOptimized
            for (int i = 0; i < areaPoints.length; i++) {
                final int areaIndex = i;
                
                try {
                    //thread 1: mov & pos
                    CompletableFuture<Void> movementFuture = CompletableFuture.runAsync(() -> {
                        try {
                            Point targetPoint = areaPoints[areaIndex];
                            if (!isPointSafe(targetPoint)) {
                                targetPoint = findNearestSafePoint(targetPoint);
                            }
                            moveToWithRetry(targetPoint, createScanningQuaternion(targetPoint));
                        } catch (Exception e) {
                            handleEmergencyStop();
                            throw e;
                        }
                    }, executor);

                    //thread 2: img proc menggunakan scanAreaOptimized
                    CompletableFuture<Boolean> processingFuture = CompletableFuture.supplyAsync(() -> {
                        return scanAreaOptimized(areaIndex, null);
                    }, executor);

                    movementFuture.get(45, TimeUnit.SECONDS);
                    if (processingFuture.get(15, TimeUnit.SECONDS)) {
                        continue;
                    }
                } catch (Exception e) {
                    handleEmergencyStop();
                    throw new RuntimeException("Area scanning failed: " + areaIndex, e);
                }
            }

            try {
                completeTargetOperationOptimized();
            } catch (Exception e) {
                handleEmergencyStop();
                throw new RuntimeException("Target operation failed", e);
            }
            
            api.reportRoundingCompletion();

        } catch (Exception e) {
            e.printStackTrace();
            handleMissionFailure();
        } finally {
            cleanup();
        }
    }


    private boolean scanAreaOptimized(int areaIndex, DetectorParameters parameters) {
        try {
            api.flashlightControlFront(0.01f);
            
            float[] angles = calculateOptimalScanAngles(areaPoints[areaIndex]);
            
            for (float angle : angles) {
                Quaternion rotatedQuat = adjustQuaternionByDegrees(
                    createScanningQuaternion(areaPoints[areaIndex]), 
                    angle
                );
                
                if (!isValidMovement(areaPoints[areaIndex], rotatedQuat)) {
                    continue;
                }

                try {
                    CompletableFuture<Boolean> detectionFuture = CompletableFuture.supplyAsync(() -> {
                        return processImagesOptimized(areaIndex, parameters);
                    }, executor);

                    if (detectionFuture.get(2, TimeUnit.SECONDS)) {
                        Mat image = api.getMatNavCam();
                        api.saveMatImage(image, "found_item_area_" + (areaIndex + 1));
                        return true;
                    }
                } catch (TimeoutException te) {
                    continue;
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return false;
            
        } finally {
            try {
                api.flashlightControlFront(0f);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private float[] calculateOptimalScanAngles(Point target) {
        double baseAngle = Math.toDegrees(Math.atan2(
            target.getY() - startPoint.getY(),
            target.getX() - startPoint.getX()
        ));
        
        return new float[]{
            0f,
            (float)(baseAngle - 15),
            (float)(baseAngle + 15),
            (float)(baseAngle - 30),
            (float)(baseAngle + 30)
        };
    }

    private boolean processImagesOptimized(int areaIndex, DetectorParameters parameters) {
        Mat image = api.getMatNavCam();
        String cacheKey = "area_" + areaIndex;
        
        Mat gray = imageCache.computeIfAbsent(cacheKey, k -> new Mat());
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Imgproc.threshold(gray, gray, 127, 255, Imgproc.THRESH_BINARY);

        List<Integer> markers = detectArUcoMarkers(gray, "area_" + areaIndex);
        
        if (!markers.isEmpty()) {
            String detectedItem = identifyItem(markers);
            if (!"unknown".equals(detectedItem)) {
                api.setAreaInfo(areaIndex + 1, detectedItem, 1);
                foundItemsMap.put(areaIndex + 1, detectedItem);
                return true;
            }
        }
        return false;
    }

    private List<Integer> detectMarkersOptimized(Mat gray, DetectorParameters parameters) {
        List<Integer> detectedMarkers = new ArrayList<>();
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        try {
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            Aruco.detectMarkers(
                gray,              
                dictionary,        
                corners,            
                ids              
            );
            
            if (!ids.empty()) {
                for (int i = 0; i < ids.rows(); i++) {
                    detectedMarkers.add((int) ids.get(i, 0)[0]);
                }

            }
        } finally {
            ids.release();
            corners.forEach(Mat::release);
        }
        
        return detectedMarkers;
    }

    private void handleMissionFailure() {
        Point currentPos = api.getRobotKinematics().getPosition();
        Quaternion currentQuat = api.getRobotKinematics().getOrientation();
        Point safePoint = findNearestSafePoint(currentPos);
        moveToWithRetry(safePoint, currentQuat);
    }

    private void cleanup() {
        try {
            shutdownLights();
            releaseImageResources();
            shutdownExecutor();
            
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                api.shutdownFactory();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private void shutdownLights() {
        try {
            api.flashlightControlFront(0f);
            api.flashlightControlBack(0f);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void releaseImageResources() {
        try {
            if (cameraMatrix != null) {
                cameraMatrix.release();
                cameraMatrix = null;
            }
            if (distCoeffs != null) {
                distCoeffs.release();
                distCoeffs = null;
            }
            
            if (imageCache != null) {
                imageCache.values().forEach(mat -> {
                    if (mat != null && !mat.empty()) {
                        mat.release();
                    }
                });
                imageCache.clear();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void shutdownExecutor() {
        if (executor != null && !executor.isShutdown()) {
            try {
                executor.shutdown();
                if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                    executor.shutdownNow();
                    executor.shutdownNow();
                }
            } catch (InterruptedException e) {
                executor.shutdownNow();
                Thread.currentThread().interrupt();
            }
        }
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

        Kinematics currentKinematics = api.getRobotKinematics();
        
        for (Point waypoint : safeWaypoints) {
            Result moveTest = api.moveTo(waypoint, currentKinematics.getOrientation(), false);
            
            if (moveTest != null && isPointSafe(waypoint)) {
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

    //helper mt to handle movement with retry logic
    private void moveToWithRetry(Point targetPoint, Quaternion targetQuaternion) {
        try {
            double MAX_SAFE_DISTANCE = 2.0; //meters
            
            if (distance(api.getRobotKinematics().getPosition(), targetPoint) > MAX_SAFE_DISTANCE) {
                handleEmergencyStop();
                throw new RuntimeException("Movement distance exceeds safety limit");
            }
            
            int maxRetries = 3;
            int retryCount = 0;
            boolean success = false;
            
            Point currentPosition = api.getRobotKinematics().getPosition();
            Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();
            
            if (!isValidDistance(currentPosition, targetPoint) || 
                !isValidRotation(currentQuaternion, targetQuaternion)) {
                handleEmergencyStop();
                throw new RuntimeException("Invalid movement parameters");
            }
            
            while (!success && retryCount < maxRetries) {
                try {
                    List<Point> path = findPathAStar(currentPosition, targetPoint);
                    
                    if (path == null || path.isEmpty()) {
                        throw new RuntimeException("No valid path found");
                    }
                    
                    for (Point waypoint : path) {
                        if (!isValidDistance(currentPosition, waypoint)) {
                            throw new RuntimeException("Invalid distance to waypoint");
                        }
                        
                        //calculate intermediate quaternion for smooth rotation
                        Quaternion intermediateQuat = calculateIntermediateQuaternion(
                            currentQuaternion, 
                            targetQuaternion, 
                            currentPosition, 
                            waypoint
                        );

                        if (!isValidRotation(currentQuaternion, intermediateQuat)) {
                            throw new RuntimeException("Invalid rotation to waypoint");
                        }
                        
                        //execute movement with all safety checks
                        Result moveResult = api.moveTo(waypoint, intermediateQuat, true);
                        if (moveResult == null) {
                            throw new RuntimeException("Movement failed");
                        }
                        
                        double distance = distance(currentPosition, waypoint);
                        double moveTime = distance / MAX_VELOCITY;
                        Thread.sleep((long)(moveTime * 1000) + 1000); 
                        
                        currentPosition = waypoint;
                        currentQuaternion = intermediateQuat;
                    }
                    
                    //final movement to target with exact quaternion
                    if (!isValidRotation(currentQuaternion, targetQuaternion)) {
                        api.moveTo(currentPosition, targetQuaternion, true);
                        Thread.sleep(1000); //final rotation
                    }
                    
                    success = true;
                    
                } catch (Exception e) {
                    retryCount++;
                    if (retryCount >= maxRetries) {
                        handleEmergencyStop();
                        throw new RuntimeException("Failed to move after " + maxRetries + " attempts", e);
                    }
                    Thread.sleep(1000);
                }
            }
        } catch (Exception e) {
            handleEmergencyStop();
            throw new RuntimeException("Movement failed with emergency", e);
        }
    }

    //helper mt to calculate quaternion interpolation
    private Quaternion calculateIntermediateQuaternion(
            Quaternion start, 
            Quaternion end, 
            Point currentPos, 
            Point targetPos) {
        
        //calculate progress along path
        double totalDistance = distance(currentPos, targetPos);
        double progress = Math.min(1.0, distance(currentPos, targetPos) / totalDistance);
        
        //interpolate quaternion
        return new Quaternion(
            (float)(start.getX() + (end.getX() - start.getX()) * progress),
            (float)(start.getY() + (end.getY() - start.getY()) * progress),
            (float)(start.getZ() + (end.getZ() - start.getZ()) * progress),
            (float)(start.getW() + (end.getW() - start.getW()) * progress)
        );
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

    @Override
    protected void runPlan2() {
        // Not used in preliminary round
    }

    @Override
    protected void runPlan3() {
        // Not used in preliminary round
    }

    private Point findNearestSafePoint(Point currentPos) {
        Point nearest = safeWaypoints[0];
        double minDist = Double.MAX_VALUE;
        
        for (Point waypoint : safeWaypoints) {
            double dist = distance(waypoint, currentPos);
            if (dist < minDist && isPointSafe(waypoint)) {
                minDist = dist;
                nearest = waypoint;
            }
        }
        return nearest;
    }

    private boolean isValidMovement(Point point, Quaternion quat) {
        return isPointSafe(point) && 
               isValidRotation(api.getRobotKinematics().getOrientation(), quat);
    }

    private String identifyItem(List<Integer> markers) {
        if (markers == null || markers.isEmpty()) {
            return "unknown";
        }
        
        for (Integer markerId : markers) {
            String item = markerToItem.get(markerId);
            if (item != null) {
                return item;
            }
        }
        return "unknown";
    }

    private Point getCorrectedPointForNavCam(Point original) {
        return new Point(
            original.getX() + NAV_CAM_OFFSET[0],
            original.getY() + NAV_CAM_OFFSET[1],
            original.getZ() + NAV_CAM_OFFSET[2]
        );
    }

    private List<Integer> detectArUcoMarkers(Mat image, String debugTag) {
        List<Integer> detectedMarkers = new ArrayList<>();
        Mat gray = new Mat();
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        
        try {
            Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            
            Aruco.detectMarkers(gray, dictionary, corners, ids);
            
            if (!ids.empty()) {
                for (int i = 0; i < ids.rows(); i++) {
                    detectedMarkers.add((int) ids.get(i, 0)[0]);
                }
            }
            return detectedMarkers;
            
        } finally {
            gray.release();
            ids.release();
            corners.forEach(Mat::release);
        }
    }

    private void saveDebugImage(Mat image, String tag, int attempt) {
        api.saveMatImage(image, tag + "_" + attempt);
    }

    private void completeTargetOperationOptimized() {
        String targetItem = "unknown";
        Point correctedAstronautPoint = getCorrectedPointForNavCam(astronautPoint);
        
        moveToWithRetry(correctedAstronautPoint, astronautQuaternion);
        
        for (int attempt = 0; attempt < 5 && "unknown".equals(targetItem); attempt++) {
            Mat astronautImage = api.getMatNavCam();
            List<Integer> targetMarkers = detectArUcoMarkers(astronautImage, 
                String.format("astronaut_target_attempt_%d", attempt));
            targetItem = identifyItem(targetMarkers);
            
            if (!"unknown".equals(targetItem)) {
                api.saveMatImage(astronautImage, "target_item_detected_" + attempt);
                break;
            }
        }
        
        api.notifyRecognitionItem();
        
        if (targetAreaId != -1) {
            Point targetPoint = areaPoints[targetAreaId - 1];
            Point correctedTargetPoint = getCorrectedPointForNavCam(targetPoint);
            
            if (isPointSafe(targetPoint)) {
                moveToWithRetry(correctedTargetPoint, createScanningQuaternion(targetPoint));
                Mat finalImage = api.getMatNavCam();
                api.saveMatImage(finalImage, "final_target");
                api.takeTargetItemSnapshot();
                api.reportRoundingCompletion();
            }
        }
    }


    private List<Point> findPathAStar(Point start, Point goal) {
        try {
            PriorityQueue<Node> openSet = new PriorityQueue<>();
            Set<String> closedSet = new HashSet<>();
            int maxIterations = 1000;
            
            Node startNode = new Node(start, null, 0, heuristic(start, goal));
            openSet.add(startNode);
            
            while (!openSet.isEmpty() && maxIterations-- > 0) {
                Node current = openSet.poll();
                String currentKey = pointToKey(current.point);
                
                if (isCloseEnough(current.point, goal)) {
                    return smoothPath(reconstructPath(current));
                }
                
                if (closedSet.contains(currentKey)) continue;
                closedSet.add(currentKey);
                
                List<Point> neighbors = getOptimizedNeighbors(current.point, goal);
                
                for (Point neighbor : neighbors) {
                    String neighborKey = pointToKey(neighbor);
                    if (closedSet.contains(neighborKey)) continue;
                    
                    double newCost = current.gCost + distance(current.point, neighbor);
                    double hCost = heuristic(neighbor, goal) * 1.1; // Weight heuristic lebih tinggi
                    
                    Node neighborNode = new Node(neighbor, current, newCost, hCost);
                    
                    Result moveTest = api.moveTo(neighbor, 
                        api.getRobotKinematics().getOrientation(), false);
                    
                    if (moveTest != null && isPointSafe(neighbor)) {
                        openSet.add(neighborNode);
                    }
                }
            }
            
            if (isPathSafe(start, goal)) {
                List<Point> directPath = new ArrayList<>();
                directPath.add(start);
                directPath.add(goal);
                return directPath;
            }
            
            return null;
        } catch (Exception e) {
            handleEmergencyStop();
            throw new RuntimeException("Path finding failed", e);
        }
    }

    private List<Point> getOptimizedNeighbors(Point current, Point goal) {
        List<Point> neighbors = new ArrayList<>();
        double step = 0.2; // 20cm steps
        
        double dx = goal.getX() - current.getX();
        double dy = goal.getY() - current.getY();
        double dz = goal.getZ() - current.getZ();
        
        //normalize
        double length = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (length > 0) {
            dx /= length;
            dy /= length;
            dz /= length;
        }
        
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if (i == 0 && j == 0 && k == 0) continue;
                    
                    double weight = (i*dx + j*dy + k*dz + 1) / 2;
                    if (weight > 0.3) {
                        Point neighbor = new Point(
                            current.getX() + i * step,
                            current.getY() + j * step,
                            current.getZ() + k * step
                        );
                        neighbors.add(neighbor);
                    }
                }
            }
        }
        
        return neighbors;
    }

    private String pointToKey(Point p) {
        return String.format("%.2f,%.2f,%.2f", p.getX(), p.getY(), p.getZ());
    }

    private boolean isCloseEnough(Point p1, Point p2) {
        return distance(p1, p2) < 0.05; //5cm threshold
    }

    private double heuristic(Point start, Point goal) {
        return distance(start, goal);
    }

    private List<Point> reconstructPath(Node endNode) {
        List<Point> path = new ArrayList<>();
        Node current = endNode;
        
        while (current != null) {
            path.add(0, current.point);
            current = current.parent;
        }
        
        return smoothPath(path);
    }

    private List<Point> smoothPath(List<Point> path) {
        if (path.size() <= 2) return path;
        
        List<Point> smoothed = new ArrayList<>();
        smoothed.add(path.get(0));
        
        int i = 0;
        while (i < path.size() - 1) {
            int j = path.size() - 1;
            while (j > i) {
                if (isPathSafe(path.get(i), path.get(j)) && 
                    isValidDistance(path.get(i), path.get(j)) &&
                    isPointSafe(path.get(j))) {
                    smoothed.add(path.get(j));
                    i = j;
                    break;
                }
                j--;
            }
            if (j == i) {
                i++;
                if (i < path.size()) {
                    smoothed.add(path.get(i));
                }
            }
        }
        
        return smoothed;
    }

    private void handleEmergencyStop() {
        try {
            Kinematics currentKinematics = api.getRobotKinematics();
            Point currentPos = currentKinematics.getPosition();
            Quaternion currentQuat = currentKinematics.getOrientation();
            
            Point safePoint = findNearestSafePoint(currentPos);
            
            Result moveResult = api.moveTo(safePoint, currentQuat, true);
            
            if (moveResult == null) {
                Point relativeMove = new Point(
                    safePoint.getX() - currentPos.getX(),
                    safePoint.getY() - currentPos.getY(),
                    safePoint.getZ() - currentPos.getZ()
                );
                api.relativeMoveTo(relativeMove, currentQuat, true);
            }
            
            api.setAreaInfo(999, "EMERGENCY_STOP", 1); 
            
        } catch (Exception e) {
            e.printStackTrace();
            api.setAreaInfo(999, "ERROR: " + e.getMessage(), 0);
        } finally {
            cleanup();
        }
    }
}