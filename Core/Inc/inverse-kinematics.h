/*
 * inverse-kinematics.h
 *
 *  Created on: Feb 13, 2025
 *      Author: muhta
 */

#ifndef INC_INVERSE_KINEMATICS_H_
#define INC_INVERSE_KINEMATICS_H_

#include "stm32f4xx_hal.h"
#include "Eigen/Dense"

using namespace Eigen;


MatrixXd inverse_kinematics(int Theta, int Phi, int Pz);
VectorXd row_norms(const MatrixXd &mat);
void print_mat_uart(UART_HandleTypeDef *huart, const MatrixXd &mat);
void print_vector_uart(UART_HandleTypeDef *huart, const VectorXd &vect);

#endif /* INC_INVERSE_KINEMATICS_H_ */
