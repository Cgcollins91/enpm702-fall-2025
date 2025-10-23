/**
 * @file    main.cpp
 * @author  Chris Collins
 * @brief   Simulates a LIDAR, Camera, and IMU Sensor system on a robot by generating randomn sensor data and 
 *          processes the generated data to perform validity checks, quality assessments, and generates report
 *          of each timestamp's sensor data and summarizes 
 * @version 1.0
 * @date    10-10-2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "sensor_types.hpp"
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

int main() {
  // Storage for all sensor data across timestamps
  std::vector<TimestampData> sensor_readings;
  sensor_readings.reserve(5);

  // Quality tracking variables
  const std::unordered_map<std::string, int> initial_counts{{"LIDAR", 0},
                                                            {"Camera", 0},
                                                            {"IMU", 0}};
  std::unordered_map<std::string, int> valid_readings{initial_counts};
  std::unordered_map<std::string, int> total_readings{initial_counts};

  // Variables for calculating summary statistics across timestamps
  double total_lidar_avg_distance{0.0};   // Total of average LIDAR distances   
  double total_camera_avg_brightness{0.0};// Total of average Camera brightness 
  double total_imu_rotation{0.0};         // Total of average IMU rotation      
  int    total_obstacles_detected{0};     // Total obstacles detected         (Count of LIDAR readings < obstacle_threshold in sensor_types.hpp) 
  int    day_mode_count{0};               // Count of day mode detections     (Brightness > day_night_threshold in sensor_types.hpp)    
  int    night_mode_count{0};             // Count of night mode detections   (Brightness <= day_night_threshold in sensor_types.hpp)    
  int    stable_imu_count{0};             // Count of stable IMU detections   (All Roll, Pitch, Yaw <= imu_stability_threshold in sensor_types.hpp)     
  int    unstable_imu_count{0};           // Count of unstable IMU detections (All Roll, Pitch, Yaw > imu_stability_threshold in sensor_types.hpp)
  double sum_valid_readings{0.0};         // Sum of valid readings across all sensors
  double sum_total_readings{0.0};         // Sum of total readings across all sensors

  // Variables for current timestamp processing
  constexpr int time_steps{5};              // Number of timestamps to simulate
  std::string   lidar_status{};             // Status string ("GOOD" or "POOR") for LIDAR current timestamp
  std::string   camera_status{};            // Status string ("GOOD" or "POOR") for Camera current timestamp
  std::string   camera_lighting_mode{};     // Lighting mode string ("DAY" or "NIGHT") for Camera current timestamp
  std::string   imu_stability{};            // Stability string ("STABLE" or "UNSTABLE") for IMU current timestamp
  std::string   imu_status{};               // Status string ("GOOD" or "POOR") for IMU current timestamp
  double        lidar_avg_distance{0.0};    // Average LIDAR distance for current timestamp
  double        obstacles_detected{0.0};    // Number of obstacles detected for current timestamp
  double        avg_camera_brightness {0.0};// Average Camera brightness for current timestamp
  double        imu_rotation{0.0};          // Total IMU rotation for current timestamp

  std::cout << "=== ROBOT DUAL-SENSOR SYSTEM ===\n\n";

  // ========================================================================
  // Step 1: Data Generation and Storage
  // ========================================================================

  // Create random number generator for simulated sensor readings
  std::random_device rd;
  std::mt19937 gen{rd()};

  // Define sensor reading distributions
  std::uniform_real_distribution<double> lidar_distrib{lidar_min_range, lidar_max_range};
  std::uniform_int_distribution<int>     camera_distrib{rgb_min, rgb_max};  
  std::uniform_real_distribution<double> imu_distrib{imu_min_rotation, imu_max_rotation};

  for(int i=0; i < time_steps; ++i ){
    std::vector<double> lidar_readings{};             // Initialize LIDAR measurement vector
    TimestampData temp;                               // Temporary storage for current timestamp data
    
    for(int j{0}; j < lidar_readings_count; ++j){
      lidar_readings.push_back(lidar_distrib(gen));   // Generate 8 random LIDAR measurements
    }

    std::tuple<int, int, int> camera_readings{        // Get random RGB values for Camera reading
          camera_distrib(gen),   // Red
          camera_distrib(gen),   // Green
          camera_distrib(gen)};  // Blue 

    std::tuple<double, double, double> imu_readings{  // Get random Roll, Pitch, Yaw values for IMU reading
          imu_distrib(gen),      // Roll
          imu_distrib(gen),      // Pitch
          imu_distrib(gen)};     // Yaw 

    temp.lidar_readings  = lidar_readings;            // Store LIDAR, Camera, IMU, and timestamp in temp struct
    temp.camera_readings = camera_readings;
    temp.imu_readings    = imu_readings;
    temp.timestamp       = i;
    sensor_readings.push_back(temp);                  // Append temp timestamp struct to main storage vector
  }
  
  // ========================================================================
  // Step 2: Data Processing Loop
  // ========================================================================
  std::cout << "Generating Sensor Data for " << time_steps <<" Timestamps..." << "\n\n";
  for (const auto& data : sensor_readings){ // Loop through each timestamp's sensor data with non-copying reference

    // Reset current timestamp processing variables and set default values
    lidar_avg_distance       = 0;
    obstacles_detected       = 0;
    lidar_status             = "GOOD";
    camera_status            = "GOOD";
    camera_lighting_mode     = "DAY";
    imu_status               = "GOOD";

    total_readings["Camera"]++; // Increment total reading counts for each sensor type
    total_readings["LIDAR"]++;
    total_readings["IMU"]++;


    // ========================================================================
    // Step 3: Sensor-Specific Processing
    // ========================================================================

    std::cout << "Processing Timestamp: " << data.timestamp << '\n'; // Print Current Timestamp and 
    std::cout << std::fixed << std::setprecision(2);                 // Set precision to 2 decimal places for LIDAR and Camera output

    lidar_avg_distance = std::accumulate(data.lidar_readings.begin(), data.lidar_readings.end(), 0.0 ) / 
                         data.lidar_readings.size() ;   // Calculate average LIDAR distance for current timestamp

    const auto [r, g, b]  = data.camera_readings;       // Unpack RGB values from Camera reading tuple            
    avg_camera_brightness = (r + g + b) / 3.0;          // Calculate average brightness for current timestamp

    if (avg_camera_brightness <= day_night_threshold){  // Determine if Camera is in DAY or NIGHT mode based on 
      camera_lighting_mode = "NIGHT";                   // brightness relative to threshold
      night_mode_count++;                               // Increment night mode detection count
    } else day_mode_count++;                            // Otherwise keep default DAY mode and increment day mode detection count                

    const auto [roll, pitch, yaw] = data.imu_readings; // Unpack Roll, Pitch, Yaw values from IMU reading tuple
    imu_rotation                  = pow( pow(roll,2) + pow(pitch,2) + pow(yaw,2), 0.5); // Calculate total rotation for current timestamp


    // ========================================================================
    // Step 4: Quality Assessment and Status Determination
    // ========================================================================
    

    // ======================================================
    // Start Print Lidar Readings, Determine Status and Obstacle Count
    // ======================================================
    
    std::cout << "- LIDAR: [";
    for (const auto& lidar_reading : data.lidar_readings){             // Loop through LIDAR readings using non-copying reference 
      
      // Print LIDAR measurements with comma seperation except last value
      if (&lidar_reading != &data.lidar_readings.back()){
               std::cout << lidar_reading << ", ";
      } else { std::cout << lidar_reading;}                              // Don't print comma after last value

      if (lidar_reading <= obstacle_threshold) {obstacles_detected++;}  // Count obstacles within threshold distance
      if (lidar_reading < lidar_min_valid)    {lidar_status = "POOR";} // If any reading below min valid, set status to POOR

    }
    std::cout << "] \n";

    // =====================================================
    // END Print LIDAR Readings and Determine Status and Obstacle Count
    // =====================================================
  
    if (lidar_status == "GOOD") {valid_readings["LIDAR"]++;}         // Increment valid LIDAR reading count if status is GOOD

    if (avg_camera_brightness < brightness_threshold){               // If brightness < brightness_threshold,
       camera_status = "POOR";                                       // set Camera status to POOR, otherwise keep default GOOD                         
    } else valid_readings["Camera"]++;                               // and Increment valid Camera reading count 

    if ( (abs(roll)  < imu_stability_threshold) &&                   // If all Roll, Pitch, Yaw below stability threshold, 
         (abs(pitch) < imu_stability_threshold) &&                    
         (abs(yaw)   < imu_stability_threshold)){     
      imu_stability = "STABLE";                                      // set IMU stability to STABLE          
      stable_imu_count++;                                            // and increment stable IMU detection count                 
    }else{
      imu_stability = "UNSTABLE";                                    // Otherwise set IMU stability to UNSTABLE
      unstable_imu_count++;                                          // and increment unstable IMU detection count
    } 

    if ( (imu_min_rotation < roll)  && (roll  < imu_max_rotation) && // If all Roll, Pitch, Yaw within valid rotation range
         (imu_min_rotation < pitch) && (pitch < imu_max_rotation) && // increment valid IMU reading and keep default GOOD status
         (imu_min_rotation < yaw)   && (yaw   < imu_max_rotation) ){
          valid_readings["IMU"]++;
    }else imu_status = "POOR";                                       // Otherwise set IMU status to POOR

    // Print LIDAR summary for current timestamp
    std::cout << "      Avg: " << lidar_avg_distance << "m, Obstacles: " << std::setprecision(0) 
              << obstacles_detected << ", STATUS: "  << lidar_status     << '\n';

    // Print Current Timestamp Camera measurement, brightness, mode, and status
    std::cout << std::setprecision(1) << "- Camera: RGB(" << r << ", " << g << ", " << b << "), Brightness: " 
              << avg_camera_brightness << ", Mode: " << camera_lighting_mode 
              << ", Status: "          << camera_status << '\n';

    // Print Current Timestamp IMU measurement, total rotation, stability, and status
    std::cout << "- IMU: RPY("      << roll << ", " << pitch   << ", " << yaw 
              << "), Total Rotation: "              << imu_rotation 
              << " deg, Mode: "                     << imu_stability 
              << ", Status: "                       << imu_status << "\n\n";

    // Accumulate Totals for summary statistics
    total_camera_avg_brightness += avg_camera_brightness;
    total_lidar_avg_distance    += lidar_avg_distance;
    total_imu_rotation          += imu_rotation;
    total_obstacles_detected    += obstacles_detected;
  }

  // ========================================================================
  // STEP 5: Summary Statistics and Display
  // ========================================================================

  sum_total_readings = total_readings["Camera"] + total_readings["IMU"] + total_readings["LIDAR"];
  sum_valid_readings = valid_readings["Camera"] + valid_readings["IMU"] + valid_readings["LIDAR"];

  // Print Total Valid and Total Readings Processed, set precision to 2 decimal places for remainder of output
  std::cout << std::setprecision(2) << "=== SUMMARY STATISTICS ===\n"
            << "Total Readings Processed: "       << sum_total_readings << '\n'
            << "Valid Readings: "                 << sum_valid_readings << " ("
            << (static_cast<double>(sum_valid_readings) / sum_total_readings)*100.0    << "%) \n\n";

  // Print Sensor Reliability Report (valid vs total readings and % Valid)
  std::cout << "Sensor Reliability Report:" << '\n'
            << "- LIDAR:   " << valid_readings["LIDAR"]    << "/" << time_steps   << "(" 
            << (static_cast<double>(valid_readings["LIDAR"])   /     time_steps)*100.0 << "%)\n"

            << "- Camera:  " << valid_readings["Camera"]   << "/" << time_steps   << "("
            << (static_cast<double>(valid_readings["Camera"])  /     time_steps)*100.0   << "%)\n"

            << "- IMU:     " << valid_readings["IMU"]      << "/" << time_steps   << "("
            << (static_cast<double>(valid_readings["IMU"])     /     time_steps)*100.0 << "%)\n\n";

  // Print Operational Statistics: Average and count across timestamps)
  std::cout << "Operational Statistics: \n"
            << "Average LIDAR Distance:     " << total_lidar_avg_distance    / time_steps << "m \n"
            << "Total Obstacles Detected:   " << total_obstacles_detected                 << '\n' << std::setprecision(1) 
            << "Average Camera Brightness:  " << total_camera_avg_brightness / time_steps << '\n'
            << "   - Day   Mode Detections: " << day_mode_count                           << '\n'
            << "   - Night Mode Detections: " << night_mode_count                         << '\n'
            << "Average IMU Total Rotation: " << total_imu_rotation          / time_steps << " deg \n"
            << "   -   Stable Detections:   " << stable_imu_count                         << '\n'
            << "   - Unstable Detections:   " << unstable_imu_count                       << '\n';

std::cout   << "\n === END OF PROGRAM === \n";
}