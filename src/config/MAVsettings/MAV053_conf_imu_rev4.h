/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file MAV051_conf_imu_rev4.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief This file defines the mapping between the IMU and the compass and the 
 * frames of the vehicles as well as the scales and the biaises. 
 * The NED frame is used.
 *
 ******************************************************************************/


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_

#define GYRO_AXIS_X 0									///< Gyroscope x axis
#define GYRO_AXIS_Y 1									///< Gyroscope y axis
#define GYRO_AXIS_Z 2									///< Gyroscope z axis

#define ACC_AXIS_X 0										///< Accelerometer x axis
#define ACC_AXIS_Y 1										///< Accelerometer y axis
#define ACC_AXIS_Z 2										///< Accelerometer z axis

#define MAG_AXIS_X 2										///< Compass x axis
#define MAG_AXIS_Y 0										///< Compass y axis
#define MAG_AXIS_Z 1										///< Compass z axis

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111f
#define RAW_GYRO_X_SCALE 818.5111f						///< Gyroscope x axis scale
#define RAW_GYRO_Y_SCALE 818.5111f					///< Gyroscope y axis scale
#define RAW_GYRO_Z_SCALE 818.5111f					///< Gyroscope z axis scale

#define GYRO_BIAIS_X 0.0f							///< Gyroscope x biais
#define GYRO_BIAIS_Y 0.0f								///< Gyroscope y biais
#define GYRO_BIAIS_Z 0.0f								///< Gyroscope z biais

#define GYRO_ORIENTATION_X  1.0f								///< Gyroscope x axis direction
#define GYRO_ORIENTATION_Y -1.0f								///< Gyroscope y axis direction
#define GYRO_ORIENTATION_Z -1.0f								///< Gyroscope z axis direction

#define RAW_ACC_X_SCALE 4034.0f							///< Accelerometer x axis scale
#define RAW_ACC_Y_SCALE 4064.7f							///< Accelerometer y axis scale
#define RAW_ACC_Z_SCALE 4112.3f							///< Accelerometer z axis scale

#define ACC_BIAIS_X -95.0f	  								///< Accelerometer x axis biais
#define ACC_BIAIS_Y 40.0f 								///< Accelerometer y axis biais
#define ACC_BIAIS_Z 180.0f 									///< Accelerometer z axis biais

#define ACC_ORIENTATION_X  1.0f									///< Accelerometer x axis direction
#define ACC_ORIENTATION_Y -1.0f									///< Accelerometer y axis direction
#define ACC_ORIENTATION_Z -1.0f									///< Accelerometer z axis direction

#define RAW_MAG_X_SCALE 475.35f						///< Compass x axis scale
#define RAW_MAG_Y_SCALE 475.08f						///< Compass y axis scale
#define RAW_MAG_Z_SCALE 437.55f						///< Compass z axis scale

#define MAG_BIAIS_X 138.0f 								///< Compass x axis biais
#define MAG_BIAIS_Y -286.0f 								///< Compass y axis biais
#define MAG_BIAIS_Z 2.47f 							///< Compass z axis biais

#define MAG_ORIENTATION_X -1.0f									///< Compass x axis direction
#define MAG_ORIENTATION_Y -1.0f								///< Compass y axis direction
#define MAG_ORIENTATION_Z -1.0f									///< Compass z axis direction

#endif /* CONF_IMU_REV4_H_ */