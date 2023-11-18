/*******************************************************************************
Vendor: Xilinx
Associated Filename: krnl_vadd.cpp
Purpose: Vitis vector addition example
*******************************************************************************
Copyright (C) 2019 XILINX, Inc.

This file contains confidential and proprietary information of Xilinx, Inc. and
is protected under U.S. and international copyright and other intellectual
property laws.

DISCLAIMER
This disclaimer is not a license and does not grant any rights to the materials
distributed herewith. Except as otherwise provided in a valid license issued to
you by Xilinx, and to the maximum extent permitted by applicable law:
(1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX
HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY,
INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR
FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether
in contract or tort, including negligence, or under any other theory of
liability) for any loss or damage of any kind or nature related to, arising under
or in connection with these materials, including for any direct, or any indirect,
special, incidental, or consequential loss or damage (including loss of data,
profits, goodwill, or any type of loss or damage suffered as a result of any
action brought by a third party) even if such damage or loss was reasonably
foreseeable or Xilinx had been advised of the possibility of the same.

CRITICAL APPLICATIONS
Xilinx products are not designed or intended to be fail-safe, or for use in any
application requiring fail-safe performance, such as life-support or safety
devices or systems, Class III medical devices, nuclear facilities, applications
related to the deployment of airbags, or any other applications that could lead
to death, personal injury, or severe property or environmental damage
(individually and collectively, "Critical Applications"). Customer assumes the
sole risk and liability of any use of Xilinx products in Critical Applications,
subject only to applicable laws and regulations governing limitations on product
liability.

THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT
ALL TIMES.

*******************************************************************************/

//------------------------------------------------------------------------------
//
// kernel:  vadd
//
// Purpose: Demonstrate Vector Add Kernel
//

#include "hls_math.h"
#define INPUT_LENGTH 10000

#define BATCH_SIZE 4

extern "C" {
void krnl_nbody(float *particles, 
                float *temp,
                int iterations) {

    const float time_step = 0.01;
    const float G = 6.67430e-11f;
    const float min_cul_radius = 0.25;

    float* particles_tmp;
    float* temp_tmp;
    particles_tmp = particles;
    temp_tmp = temp;

    float BufP[BATCH_SIZE][5];

    TIME_STEP: for (int t = 0; t < iterations; t++){
        


        #pragma HLS pipeline off
    Pi: for (int i = 0; i < INPUT_LENGTH; i += 5){
            //#pragma HLS unroll factor = 50
            // current particle i 
            float x1 = particles_tmp[i];
            float y1 = particles_tmp[i + 1];
            float vx = particles_tmp[i + 2];
            float vy = particles_tmp[i + 3];
            float mass = particles_tmp[i + 4];
            

            // forces are accumulated in the inner loop
            float force_x = 0;
            float force_y = 0;


            #pragma HLS pipeline off
        Pj: for (int j = 0; j < INPUT_LENGTH; j += 5)
            {
                if (i == j){continue;}
                
                // CalculateForce2D(particleData, i, j, force_x, force_y);

                // read particle j
                float x2 = particles_tmp[j];
                float y2 = particles_tmp[j + 1];
                float mass2 = particles_tmp[j + 4];

                // Calculate the distance between the two particles in 2D
                float dx = x2 - x1;
                float dy = y2 - y1;
                // fixed_t distance = static_cast<fixed_t>(sqrt(static_cast<float>(dx * dx + dy * dy)));
                float distance = sqrt(dx * dx + dy * dy);
                // float distance = 3.0f;
                //  Define gravitational constant
                //  Calculate the gravitational force in 2D

                if (distance <= min_cul_radius)
                {
                    float force_magnitude = (G * mass * mass2) / (distance * distance);
                    // Calculate force components in 2D
                    force_x += force_magnitude * (dx / distance); //dependency in accumulation
                    force_y += force_magnitude * (dy / distance); //dependency in accumulation
                }
                
            
            }

            // Calculate acceleration in 2D
            float ax = force_x / mass;
            float ay = force_y / mass;
            // Update velocity in 2D using the calculated acceleration and time step
            temp_tmp[i + 2] = vx + ax * time_step;
            temp_tmp[i + 3] = vy + ay * time_step;
            temp_tmp[i] = x1 + temp_tmp[i + 2] * time_step;
            temp_tmp[i + 1] = y1 + temp_tmp[i + 3] * time_step;
            temp_tmp[i + 4] = mass;
        }



        float *placeholder = temp_tmp;
        temp_tmp = particles_tmp;
        particles_tmp = placeholder;

    }

}
}
