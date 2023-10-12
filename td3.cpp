#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* newArray = new double[new_size];
    for(int i = 0; i < length; i++)
        newArray[i] = array[i];

    for(int i = length; i < new_size; i++)
        newArray[i] = 0;

    delete[] array;

    return newArray;
}

double* shrink_array(double* array, int length, int new_size) {
    double* newArray = new double[new_size];
    for(int i = 0; i < new_size; i++)
        newArray[i] = array[i];

    delete[] array;

    return newArray;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if(current_size == max_size){
        array = extend_array(array, max_size, max_size + 5);
        max_size += 5;
    }
    array[current_size++] = element;
    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size--;
    if(max_size - current_size >= 5){
        array = shrink_array(array, current_size, current_size);
        max_size = current_size;
    }
    return array; // YOU CAN CHANGE THIS
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

    bool hit_target, hit_obstacle;
    double v0_x, v0_y, x, y, t;
    double PI = 3.14159265;
    double g = 9.8;

    v0_x = magnitude * cos(angle * PI / 180);
    v0_y = magnitude * sin(angle * PI / 180);

    t = 0;
    x = 0;
    y = 0;

    hit_target = false;
    hit_obstacle = false;

    //set up telemetry
    telemetry = new double[5];
    telemetry_current_size = 0;
    telemetry_max_size = 5;

    while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
        double * target_coordinates = find_collision(x, y, targets, tot_targets);

        //append to telemetry
        telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

        if (target_coordinates != NULL) {
            remove_target(targets, tot_targets, target_coordinates);
            hit_target = true;
        }
        else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
            hit_obstacle = true;
        }
        else {
            t = t + simulation_interval;
            y = v0_y * t  - 0.5 * g * t * t;
            x = v0_x * t;
        }
    }

    return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

    global_telemetry = new double[5];
    global_telemetry_current_size = 0;
    global_telemetry_max_size = 5;

    int* index_telemetries = new int[tot_telemetries];
    bool has_elements_left = false;
    for(int i = 0; i < tot_telemetries; i++){
        index_telemetries[i] = 0;
        has_elements_left = has_elements_left || (index_telemetries[i] < telemetries_sizes[i]);
    }

    while(has_elements_left){
        double min_time = -1;
        int min_time_position = -1;

        for(int i = 0; i < tot_telemetries; i++){
            if(index_telemetries[i] < telemetries_sizes[i] && (min_time == -1 || telemetries[i][index_telemetries[i]] < min_time)){
                min_time = telemetries[i][index_telemetries[i]];
                min_time_position = i;
            }
        }

        if(min_time_position == -1){
            cout<<"has_elements_left should be false!\n";
            has_elements_left = false;
            break;
        }

        int cur_index = index_telemetries[min_time_position];
        if(cur_index + 2 >= telemetries_sizes[min_time_position]){
            cout<<"ERROR! FOUND TIME BUT NOT ENOUGH POSITIONS IN TELEMETRIES SIZES!\n";
            return;
        }

        global_telemetry = append_to_array(telemetries[min_time_position][cur_index], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        global_telemetry = append_to_array(telemetries[min_time_position][cur_index + 1], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        global_telemetry = append_to_array(telemetries[min_time_position][cur_index + 2], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);

        index_telemetries[min_time_position] = cur_index + 3;

        //update has_elements_left
        has_elements_left = false;
        for(int i = 0; i < tot_telemetries; i++)
            has_elements_left = (has_elements_left || (index_telemetries[i] < telemetries_sizes[i]));
    }
}
