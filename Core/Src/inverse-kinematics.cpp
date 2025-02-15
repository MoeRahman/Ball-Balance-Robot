/*
 * inverse-kinematics.cpp
 *
 *  Created on: Feb 13, 2025
 *      Author: muhta
 */

#include <iostream>
#include "stm32f4xx_hal.h"
#include <inverse-kinematics.h>
#include "Eigen/Dense"
#include <cmath>

using namespace Eigen;
using namespace std;

extern UART_HandleTypeDef huart1;

MatrixXd inverse_kinematics(int Theta, int Phi, int Pz) {
    // Convert degrees to radians
    double theta = M_PI * Theta / 180.0;
    double phi = M_PI * Phi / 180.0;

    // Define A and B vectors (3x3 matrices)
    MatrixXd a(3,3), b(3,3);
    a <<  0, 100, 0,
         100 * sin(M_PI / 3), -100 * cos(M_PI / 3), 0,
        -100 * sin(M_PI / 3), -100 * cos(M_PI / 3), 0;

    b = a; // b is identical to a

    // Define Center P Vector
    Vector3d P(0, 0, Pz);

    // Roll rotation matrix (about X-axis)
    Matrix3d R_x;
    R_x <<  1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);

    // Pitch rotation matrix (about Y-axis)
    Matrix3d R_y;
    R_y <<  cos(phi), 0, sin(phi),
            0, 1, 0,
            -sin(phi), 0, cos(phi);

    Matrix3d R = R_x*R_y;

    Matrix3d transformed_b = R * b.transpose();
    MatrixXd transformed_b_T  = transformed_b.transpose(); // Ensure correct column-wise addition
    MatrixXd result = transformed_b_T.rowwise() + P.transpose() - a;
    print_mat_uart(&huart1, result);
    // Compute output
    return result;
}

// Function to compute the norms of each column
VectorXd row_norms(const MatrixXd& mat)
{
    VectorXd norms(mat.rows());  // Vector to store row norms
    for (int i = 0; i < mat.rows(); ++i) {
        norms(i) = mat.row(i).norm();  // Compute norm of each row
    }

    return norms;  // Return the first row norm, or modify this as needed
}


void print_mat_uart(UART_HandleTypeDef *huart, const MatrixXd &mat)
{
    char buffer[128];  // Buffer to store each row
    int len;

    for (int i = 0; i < mat.rows(); ++i) {
        len = 0;
        for (int j = 0; j < mat.cols(); ++j) {
            len += snprintf(buffer + len, sizeof(buffer) - len, "%.2f\t", mat(i, j));
            // Format each number as float with 2 decimal places
        }
        len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n"); // New line after row

        // Send row over UART
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
}

void print_vector_uart(UART_HandleTypeDef *huart, const VectorXd &vect)
{
    char buffer[128];  // Buffer to store each row
    int len;

    // Loop through the vector and print each element
    for (int i = 0; i < vect.size(); ++i) {
        len = 0;

        // Format each element of the vector
        len += snprintf(buffer + len, sizeof(buffer) - len, "%.2f\t", vect(i));

        // New line after each element
        if (i == vect.size() - 1) {
            len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
        }

        // Send over UART
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
}






