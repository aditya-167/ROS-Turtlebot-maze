# ROS-Turtlebot-maze
Turtlebot maze solver that senses environment through laser scans and navigates. A mini Project from Robot Ignite Academy Course
Two classes has been implemented :
  1. /src/src/project.py - Includes the algorithm for navigation and obstacle detection.
  2. /src/src/robot_control_class.py - Contains sensor data acquisition and methods.
  
# Tools:
  python
  numpy
  ROS

# Demo

![20200120_190819](https://user-images.githubusercontent.com/47297221/72735002-f9dd2200-3bc0-11ea-8ba3-5ee3ab3c2c8a.gif)















## [Rubric](https://review.udacity.com/#!/rubrics/2550/view) Points
---

### FP.1 Match 3D Objects
* Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). 
* Matches must be the ones with the highest number of keypoint correspondences.

* Solution: Function `matchBoundingBoxes` at the `camFusion_Student.cpp`
```c++
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    int p = prevFrame.boundingBoxes.size();
    int c = currFrame.boundingBoxes.size();
    int pt_counts[p][c] = { };
    for (auto it = matches.begin(); it != matches.end() - 1; ++it)     {
        cv::KeyPoint query = prevFrame.keypoints[it->queryIdx];
        auto query_pt = cv::Point(query.pt.x, query.pt.y);
        bool query_found = false;
        cv::KeyPoint train = currFrame.keypoints[it->trainIdx];
        auto train_pt = cv::Point(train.pt.x, train.pt.y);
        bool train_found = false;
        std::vector<int> query_id, train_id;
        for (int i = 0; i < p; i++) {
            if (prevFrame.boundingBoxes[i].roi.contains(query_pt))             {
                query_found = true;
                query_id.push_back(i);
             }
        }
        for (int i = 0; i < c; i++) {
            if (currFrame.boundingBoxes[i].roi.contains(train_pt))             {
                train_found= true;
                train_id.push_back(i);
            }
        }
        if (query_found && train_found)
        {
            for (auto id_prev: query_id)
                for (auto id_curr: train_id)
                     pt_counts[id_prev][id_curr] += 1;
        }
    }

    for (int i = 0; i < p; i++)
    {
         int max_count = 0;
         int id_max = 0;
         for (int j = 0; j < c; j++)
             if (pt_counts[i][j] > max_count)
             {
                  max_count = pt_counts[i][j];
                  id_max = j;
             }
          bbBestMatches[i] = id_max;
    }
    bool bMsg = true;
    if (bMsg)
        for (int i = 0; i < p; i++)
             cout << "Box " << i << " matches " << bbBestMatches[i]<< " box" << endl;
}
```

### FP.2 Compute Lidar-based TTC
* Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.
#### (Answer):
* Solution: Function `computeTTCLidar` at the `camFusion_Student.cpp`
```c++
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    double dT = 1 / frameRate;
    double laneWidth = 4.0; // assumed width of the ego lane
    vector<double> xPrev, xCurr;
    // find Lidar points within ego lane
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xPrev.push_back(it->x);
        }
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xCurr.push_back(it->x);
        }
    }
    double minXPrev = 0;
    double minXCurr = 0;
    if (xPrev.size() > 0)
    {
       for (auto x: xPrev)
            minXPrev += x;
       minXPrev = minXPrev / xPrev.size();
    }
    if (xCurr.size() > 0)
    {
       for (auto x: xCurr)
           minXCurr += x;
       minXCurr = minXCurr / xCurr.size();
    }
    // compute TTC from both measurements
    cout << "minXCurr: " << minXCurr << endl;
    cout << "minXPrev: " << minXPrev << endl;
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}
```

### FP.3 Associate Keypoint Correspondences with Bounding Boxes
* Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. 
* All matches which satisfy this condition must be added to a vector in the respective bounding box.
#### (Answer):
* Solution: Function `clusterKptMatchesWithROI` at the `camFusion_Student.cpp`
```c++
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    double dist_mean = 0;
    std::vector<cv::DMatch>  kptMatches_roi;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint kp = kptsCurr.at(it->trainIdx);
        if (boundingBox.roi.contains(cv::Point(kp.pt.x, kp.pt.y)))
            kptMatches_roi.push_back(*it);
     }
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
         dist_mean += it->distance;
    cout << "Find " << kptMatches_roi.size()  << " matches" << endl;
    if (kptMatches_roi.size() > 0)
         dist_mean = dist_mean/kptMatches_roi.size();
    else return;
    double threshold = dist_mean * 0.7;
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
    {
       if (it->distance < threshold)
           boundingBox.kptMatches.push_back(*it);
    }
    cout << "Leave " << boundingBox.kptMatches.size()  << " matches" << endl;
}
```

### FP.4 Compute Camera-based TTC
* Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
#### (Answer):
* Solution: Function `computeTTCCamera` at the `camFusion_Student.cpp`
```c++
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
            double minDist = 100.0; // min. required distance
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);
            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }
    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];   // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}
```

### FP.5 Performance Evaluation 1
* Find examples where the TTC estimate of the Lidar sensor does not seem plausible. 
* Describe your observations and provide a sound argumentation why you think this happened.
#### (Answer):
* I created a loop in code to test all possible combinations of detectors and descriptors and saved the results.
  * detectors: `SHITOMASI`, `FAST`, `BRISK`, `ORB`, `AKAZE`
  * descriptors: `BRISK`, `BRIEF`, `ORB`, `FREAK`  
  * Saved Performance Results of All Combination: please check `FP_6_Performance_Evaluation_2.csv` file [CSV file](https://github.com/studian/SFND_P4_3D_Object_Tracking/FP_6_Performance_Evaluation_2.csv).
  * Saved Results Images of All Combination: please check `./SFND_P4_3D_Object_Tracking/resultsImages/` folder.
* Lidar sensor based TTCs are almost corrected. 
* In case of 14-18 frames, by the eye, the distance of the front vehicle decreased, but the TTC increased.
* TTC from Lidar is not correct because of some outliers and some unstable points from preceding vehicle's front mirrors, those need to be filtered out.
* Some examples with wrong TTC estimate of the Lidar sensor:

Frame Number        | IMAGE               
--------------------| -------------------
14                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000014.png) 
15                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000015.png)
16                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000016.png) 
17                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000017.png) 
18                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000018.png) 

* TTC from Lidar is not correct because of Lidar points from preceding vehicle front mirrors. 
* Need to delete Lidar points from preceding vehicle front mirrors.
 

### FP.6 Performance Evaluation 2
* Run several detector / descriptor combinations and look at the differences in TTC estimation. 
* Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. 
* As with Lidar, describe your observations again and also look into potential reasons.
#### (Answer):
* I created a loop in code to test all possible combinations of detectors and descriptors and saved the results.
  * detectors: `SHITOMASI`, `FAST`, `BRISK`, `ORB`, `AKAZE`
  * descriptors: `BRISK`, `BRIEF`, `ORB`, `FREAK`  
  * Saved Results of All Combination: please check `FP_6_Performance_Evaluation_2.csv` file [CSV file](https://github.com/studian/SFND_P4_3D_Object_Tracking/FP_6_Performance_Evaluation_2.csv).
* Analysis of All Combination: please check `FP_6_Performance_Evaluation_2_analysis.xlsx` file [Excel file](https://github.com/studian/SFND_P4_3D_Object_Tracking/FP_6_Performance_Evaluation_2_analysis.xlsx).
* Certain detector/descriptor combinations, especially the `ORB` detectors, produced very unreliable camera TTC estimates.
* The TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles are: 
  * `SHITOMASI` / `FREAK`
  * `AKAZE` / `BRISK`
  * `AKAZE` / `FREAK`







