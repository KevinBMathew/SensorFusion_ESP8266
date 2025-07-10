#ifndef ENHANCED_FUZZY_TEMPERATURE_FUSION_H
#define ENHANCED_FUZZY_TEMPERATURE_FUSION_H

#include <Arduino.h>
#include <math.h>

// System Configuration
#define NUM_SENSORS 6

// --- Enhanced Fuzzy Temperature Fusion Class ---
class EnhancedFuzzyTemperatureFusion {
private:
    // Base priorities from fuzzyifier.pdf
    float base_priorities[NUM_SENSORS] = {0.10, 0.10, 0.15, 0.25, 0.15, 0.25};
    
    // Sensor groupings by type
    int sensor_pairs[3][2] = {{0, 1}, {2, 4}, {3, 5}}; // BME680, STS35, SHT45
    
    // Delta breakpoints [still_limit, moderate_limit, jump_limit]
    float delta_breakpoints[3][3] = {
        {0.10, 0.20, INFINITY}, // BME680
        {0.05, 0.15, INFINITY}, // STS35  
        {0.05, 0.15, INFINITY}  // SHT45
    };
    
    // Pair difference limits
    float intra_pair_breakpoints[3] = {0.05, 0.20, INFINITY};
    float inter_pair_breakpoints[3] = {0.05, 0.25, INFINITY};
    
    // System parameters
    float range_limits[2] = {-40, 125};
    float outlier_threshold = 1.5;
    float drift_threshold = 0.3;
    float stuck_threshold = 0.01;
    
    // Historical data storage
    float sensor_history[NUM_SENSORS][50];
    int history_index[NUM_SENSORS];
    int history_count[NUM_SENSORS];
    int outlier_count[NUM_SENSORS];
    float previous_readings[NUM_SENSORS];
    bool has_previous_readings = false;
    
    // Reliability values
    float reliability_values[6] = {0.0, 0.1, 0.3, 0.5, 0.8, 1.0}; // Zero, VeryLow, Low, Medium, High, VeryHigh
    
public:
    EnhancedFuzzyTemperatureFusion() {
        reset();
    }
    
    void reset() {
        for (int i = 0; i < NUM_SENSORS; i++) {
            history_index[i] = 0;
            history_count[i] = 0;
            outlier_count[i] = 0;
            previous_readings[i] = 25.0;
            for (int j = 0; j < 50; j++) {
                sensor_history[i][j] = 0.0;
            }
        }
        has_previous_readings = false;
    }
    
    // Triangular membership function
    void triangular_membership(float x, float points[3], float result[3]) {
        // Still/Match membership (decreasing from 0)
        if (x <= points[0]) {
            result[0] = 1.0;
        } else if (x <= points[1]) {
            result[0] = max(0.0f, (points[1] - x) / (points[1] - points[0]));
        } else {
            result[0] = 0.0;
        }
        
        // Moderate/SlightConflict membership (triangle)
        if (x <= points[0]) {
            result[1] = 0.0;
        } else if (x <= points[1]) {
            result[1] = (x - points[0]) / (points[1] - points[0]);
        } else if (x <= points[2]) {
            result[1] = (points[2] - x) / (points[2] - points[1]);
        } else {
            result[1] = 0.0;
        }
        
        // Jump/Clash membership (increasing from point[1])
        if (x <= points[1]) {
            result[2] = 0.0;
        } else if (x <= points[2]) {
            result[2] = (x - points[1]) / (points[2] - points[1]);
        } else {
            result[2] = 1.0;
        }
    }
    
    void update_sensor_history(float readings[NUM_SENSORS]) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!isnan(readings[i])) {
                sensor_history[i][history_index[i]] = readings[i];
                history_index[i] = (history_index[i] + 1) % 50;
                if (history_count[i] < 50) {
                    history_count[i]++;
                }
            }
        }
    }
    
    bool detect_outliers(float readings[NUM_SENSORS], bool outliers[NUM_SENSORS]) {
        bool any_outliers = false;
        for (int i = 0; i < NUM_SENSORS; i++) {
            outliers[i] = false;
            if (isnan(readings[i])) continue;
            
            // Calculate median of other sensors
            float others[NUM_SENSORS-1];
            int others_count = 0;
            for (int j = 0; j < NUM_SENSORS; j++) {
                if (j != i && !isnan(readings[j])) {
                    others[others_count++] = readings[j];
                }
            }
            
            if (others_count >= 2) {
                // Simple median calculation for small array
                for (int k = 0; k < others_count - 1; k++) {
                    for (int l = k + 1; l < others_count; l++) {
                        if (others[k] > others[l]) {
                            float temp = others[k];
                            others[k] = others[l];
                            others[l] = temp;
                        }
                    }
                }
                float median_others = (others_count % 2 == 0) ? 
                    (others[others_count/2 - 1] + others[others_count/2]) / 2.0 : 
                    others[others_count/2];
                
                float deviation = fabs(readings[i] - median_others);
                if (deviation > outlier_threshold) {
                    outliers[i] = true;
                    outlier_count[i]++;
                    any_outliers = true;
                }
            }
        }
        return any_outliers;
    }
    
    bool detect_stuck_sensor(int sensor_idx) {
        if (history_count[sensor_idx] < 20) return false;
        
        // Check last 20 readings for minimal change
        float min_val = sensor_history[sensor_idx][0];
        float max_val = sensor_history[sensor_idx][0];
        
        int start_idx = (history_index[sensor_idx] - min(20, history_count[sensor_idx]) + 50) % 50;
        for (int i = 0; i < min(20, history_count[sensor_idx]); i++) {
            int idx = (start_idx + i) % 50;
            min_val = min(min_val, sensor_history[sensor_idx][idx]);
            max_val = max(max_val, sensor_history[sensor_idx][idx]);
        }
        
        float max_change = max_val - min_val;
        
        // Check if other sensors are changing
        bool other_sensors_changing = false;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (i != sensor_idx && history_count[i] >= 20) {
                float other_min = sensor_history[i][0];
                float other_max = sensor_history[i][0];
                int other_start = (history_index[i] - min(20, history_count[i]) + 50) % 50;
                for (int j = 0; j < min(20, history_count[i]); j++) {
                    int idx = (other_start + j) % 50;
                    other_min = min(other_min, sensor_history[i][idx]);
                    other_max = max(other_max, sensor_history[i][idx]);
                }
                if ((other_max - other_min) > 0.1) {
                    other_sensors_changing = true;
                    break;
                }
            }
        }
        
        return (max_change < stuck_threshold && other_sensors_changing);
    }
    
    void calculate_deltas(float current_readings[NUM_SENSORS], float deltas[NUM_SENSORS]) {
        if (!has_previous_readings) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                deltas[i] = 0.0;
            }
            return;
        }
        
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!isnan(current_readings[i]) && !isnan(previous_readings[i])) {
                deltas[i] = fabs(current_readings[i] - previous_readings[i]);
            } else {
                deltas[i] = 0.0;
            }
        }
    }
    
    void calculate_pair_differences(float readings[NUM_SENSORS], float diffs[6]) {
        // Within-pair differences (dAA, dBB, dCC)
        for (int pair = 0; pair < 3; pair++) {
            int s1 = sensor_pairs[pair][0];
            int s2 = sensor_pairs[pair][1];
            if (!isnan(readings[s1]) && !isnan(readings[s2])) {
                diffs[pair] = fabs(readings[s1] - readings[s2]);
            } else {
                diffs[pair] = 0.0;
            }
        }
        
        // Between-pair differences (dAB, dAC, dBC)
        float pair_averages[3];
        bool pair_valid[3];
        for (int pair = 0; pair < 3; pair++) {
            int s1 = sensor_pairs[pair][0];
            int s2 = sensor_pairs[pair][1];
            if (!isnan(readings[s1]) && !isnan(readings[s2])) {
                pair_averages[pair] = (readings[s1] + readings[s2]) / 2.0;
                pair_valid[pair] = true;
            } else if (!isnan(readings[s1])) {
                pair_averages[pair] = readings[s1];
                pair_valid[pair] = true;
            } else if (!isnan(readings[s2])) {
                pair_averages[pair] = readings[s2];
                pair_valid[pair] = true;
            } else {
                pair_valid[pair] = false;
            }
        }
        
        // dAB, dAC, dBC
        if (pair_valid[0] && pair_valid[1]) {
            diffs[3] = fabs(pair_averages[0] - pair_averages[1]); // dAB
        } else {
            diffs[3] = 0.0;
        }
        
        if (pair_valid[0] && pair_valid[2]) {
            diffs[4] = fabs(pair_averages[0] - pair_averages[2]); // dAC
        } else {
            diffs[4] = 0.0;
        }
        
        if (pair_valid[1] && pair_valid[2]) {
            diffs[5] = fabs(pair_averages[1] - pair_averages[2]); // dBC
        } else {
            diffs[5] = 0.0;
        }
    }
    
    bool check_range_validity(float readings[NUM_SENSORS], bool valid[NUM_SENSORS]) {
        bool all_valid = true;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!isnan(readings[i])) {
                valid[i] = (readings[i] >= range_limits[0] && readings[i] <= range_limits[1]);
            } else {
                valid[i] = false;
            }
            if (!valid[i]) all_valid = false;
        }
        return all_valid;
    }
    
    void apply_enhanced_rules(float readings[NUM_SENSORS], float deltas[NUM_SENSORS], 
                             float diffs[6], bool range_valid[NUM_SENSORS], int reliabilities[NUM_SENSORS]) {
        
        // Initialize all sensors to medium reliability (3)
        for (int i = 0; i < NUM_SENSORS; i++) {
            reliabilities[i] = 3; // Medium
        }
        
        // Update historical data
        update_sensor_history(readings);
        
        // Priority 1: Critical fault detection
        
        // O1: Range check
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!range_valid[i]) {
                reliabilities[i] = 0; // Zero
                Serial.printf("WARNING: Sensor %d outside valid range: %.2f°C\n", i, readings[i]);
            }
        }
        
        // O2: Outlier detection
        bool outliers[NUM_SENSORS];
        if (detect_outliers(readings, outliers)) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (outliers[i]) {
                    reliabilities[i] = 0; // Zero
                    Serial.printf("WARNING: Sensor %d removed due to outlier: %.2f°C\n", i, readings[i]);
                }
            }
        }
        
        // O3: Stuck sensor detection
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (reliabilities[i] == 0) continue;
            if (detect_stuck_sensor(i)) {
                reliabilities[i] = 0; // Zero
                Serial.printf("WARNING: Sensor %d appears stuck\n", i);
            }
        }
        
        // Priority 2: Within-pair analysis
        for (int pair = 0; pair < 3; pair++) {
            int s1 = sensor_pairs[pair][0];
            int s2 = sensor_pairs[pair][1];
            float diff = diffs[pair];
            
            // Fuzzy membership for pair difference
            float diff_fuzzy[3];
            triangular_membership(diff, intra_pair_breakpoints, diff_fuzzy);
            
            // W1: Perfect pair agreement
            if (diff_fuzzy[0] > 0.9) { // low membership (close agreement)
                // Check if both sensors have low delta (still)
                int sensor_type = pair; // 0=BME680, 1=STS35, 2=SHT45
                float delta_fuzzy_s1[3], delta_fuzzy_s2[3];
                triangular_membership(deltas[s1], delta_breakpoints[sensor_type], delta_fuzzy_s1);
                triangular_membership(deltas[s2], delta_breakpoints[sensor_type], delta_fuzzy_s2);
                
                if (delta_fuzzy_s1[0] > 0.8 && delta_fuzzy_s2[0] > 0.8) {
                    if (reliabilities[s1] > 1) reliabilities[s1] = 5; // Very High
                    if (reliabilities[s2] > 1) reliabilities[s2] = 5; // Very High
                }
            }
            
            // W2: Conflict with one sensor static
            else if (diff_fuzzy[2] > 0.7) { // high membership (clash)
                int sensor_type = pair;
                float delta_fuzzy_s1[3], delta_fuzzy_s2[3];
                triangular_membership(deltas[s1], delta_breakpoints[sensor_type], delta_fuzzy_s1);
                triangular_membership(deltas[s2], delta_breakpoints[sensor_type], delta_fuzzy_s2);
                
                // If one sensor is static (low delta) while pair has conflict
                if (delta_fuzzy_s1[0] > 0.7 && reliabilities[s1] > 0) {
                    reliabilities[s1] = 1; // Very Low
                }
                if (delta_fuzzy_s2[0] > 0.7 && reliabilities[s2] > 0) {
                    reliabilities[s2] = 1; // Very Low
                }
            }
            
            // W3: Both sensors jumping together
            else if (diff_fuzzy[0] > 0.5) { // reasonable agreement
                int sensor_type = pair;
                float delta_fuzzy_s1[3], delta_fuzzy_s2[3];
                triangular_membership(deltas[s1], delta_breakpoints[sensor_type], delta_fuzzy_s1);
                triangular_membership(deltas[s2], delta_breakpoints[sensor_type], delta_fuzzy_s2);
                
                if (delta_fuzzy_s1[2] > 0.5 && delta_fuzzy_s2[2] > 0.5) { // both jumping
                    if (reliabilities[s1] > 1) reliabilities[s1] = 4; // High
                    if (reliabilities[s2] > 1) reliabilities[s2] = 4; // High
                }
            }
        }
        
        // Priority 3: Cross-pair intelligence
        
        // C1: Sensor type hierarchy validation
        float pair_averages[3];
        bool pair_valid[3];
        for (int pair = 0; pair < 3; pair++) {
            int s1 = sensor_pairs[pair][0];
            int s2 = sensor_pairs[pair][1];
            if (!isnan(readings[s1]) && !isnan(readings[s2])) {
                pair_averages[pair] = (readings[s1] + readings[s2]) / 2.0;
                pair_valid[pair] = true;
            } else {
                pair_valid[pair] = false;
            }
        }
        
        if (pair_valid[0] && pair_valid[1] && pair_valid[2]) {
            // Check if SHT45 contradicts BME680 but agrees with STS35
            float sht_bme_diff = fabs(pair_averages[2] - pair_averages[0]); // SHT45 vs BME680
            float sht_sts_diff = fabs(pair_averages[2] - pair_averages[1]); // SHT45 vs STS35
            
            if (sht_bme_diff > 0.5 && sht_sts_diff < 0.3) {
                // SHT45 and STS35 agree, BME680 disagrees
                for (int i = 0; i < 2; i++) {
                    int sensor_idx = sensor_pairs[2][i]; // SHT45
                    if (reliabilities[sensor_idx] > 1) reliabilities[sensor_idx] = 5; // Very High
                }
                for (int i = 0; i < 2; i++) {
                    int sensor_idx = sensor_pairs[1][i]; // STS35
                    if (reliabilities[sensor_idx] > 1) reliabilities[sensor_idx] = 4; // High
                }
                for (int i = 0; i < 2; i++) {
                    int sensor_idx = sensor_pairs[0][i]; // BME680
                    if (reliabilities[sensor_idx] > 1) reliabilities[sensor_idx] = 2; // Low
                }
            }
        }
        
        // C2: Environmental change detection
        // Fast sensors (BME680) responding first
        bool bme_jumping = false;
        for (int i = 0; i < 2; i++) {
            float delta_fuzzy[3];
            triangular_membership(deltas[sensor_pairs[0][i]], delta_breakpoints[0], delta_fuzzy);
            if (delta_fuzzy[2] > 0.7) { // high delta
                bme_jumping = true;
                break;
            }
        }
        
        if (bme_jumping) {
            // Check if cross-pair differences are not too high
            bool cross_clash = false;
            if (diffs[3] > 0.7 || diffs[4] > 0.7) { // dAB or dAC
                cross_clash = true;
            }
            
            if (!cross_clash) {
                for (int i = 0; i < 2; i++) {
                    int sensor_idx = sensor_pairs[0][i]; // BME680
                    if (reliabilities[sensor_idx] > 1) reliabilities[sensor_idx] = 4; // High
                }
            }
        }
    }
    
    float defuzzify_reliability(int linguistic_reliability) {
        if (linguistic_reliability < 0 || linguistic_reliability > 5) {
            return 0.5;
        }
        return reliability_values[linguistic_reliability];
    }
    
    void calculate_weights(int reliabilities[NUM_SENSORS], float weights[NUM_SENSORS]) {
        float total_weight = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            float numeric_reliability = defuzzify_reliability(reliabilities[i]);
            weights[i] = base_priorities[i] * numeric_reliability;
            total_weight += weights[i];
        }
        
        // Normalize weights
        if (total_weight > 0) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                weights[i] /= total_weight;
            }
        } else {
            // Equal weights if all failed
            float equal_weight = 1.0 / NUM_SENSORS;
            for (int i = 0; i < NUM_SENSORS; i++) {
                weights[i] = equal_weight;
            }
        }
    }
    
    float fuse_temperature(float readings[NUM_SENSORS]) {
        float deltas[NUM_SENSORS];
        float diffs[6];
        bool range_valid[NUM_SENSORS];
        int reliabilities[NUM_SENSORS];
        float weights[NUM_SENSORS];
        
        // Calculate inputs
        calculate_deltas(readings, deltas);
        calculate_pair_differences(readings, diffs);
        check_range_validity(readings, range_valid);
        
        // Apply fuzzy rules
        apply_enhanced_rules(readings, deltas, diffs, range_valid, reliabilities);
        
        // Calculate final weights
        calculate_weights(reliabilities, weights);
        
        // Calculate fused temperature
        float fused_temp = 0;
        float total_weight = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!isnan(readings[i]) && weights[i] > 0) {
                fused_temp += weights[i] * readings[i];
                total_weight += weights[i];
            }
        }
        
        if (total_weight > 0) {
            fused_temp /= total_weight;
        } else {
            // Fallback to simple average
            float sum = 0;
            int count = 0;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (!isnan(readings[i])) {
                    sum += readings[i];
                    count++;
                }
            }
            fused_temp = (count > 0) ? sum / count : NAN;
        }
        
        // Update previous readings
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!isnan(readings[i])) {
                previous_readings[i] = readings[i];
            }
        }
        has_previous_readings = true;
        
        // Debug output
        Serial.printf("Fuzzy Fusion - Weights: ");
        for (int i = 0; i < NUM_SENSORS; i++) {
            Serial.printf("S%d:%.3f ", i, weights[i]);
        }
        Serial.printf("-> %.2f°C\n", fused_temp);
        
        return fused_temp;
    }
    
    int get_outlier_count(int sensor_idx) {
        if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return 0;
        return outlier_count[sensor_idx];
    }
    
    void print_diagnostics() {
        Serial.println("\n=== FUZZY FUSION DIAGNOSTICS ===");
        Serial.println("Outlier Detection Statistics:");
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (outlier_count[i] > 0) {
                Serial.printf("  Sensor %d: %d outliers detected\n", i, outlier_count[i]);
            }
        }
        Serial.println("================================");
    }
};

#endif // ENHANCED_FUZZY_TEMPERATURE_FUSION_H
