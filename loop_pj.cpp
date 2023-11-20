void nbody_loop_pj(float BufP[BATCH_SIZE][5], 
                    float BufF[BATCH_SIZE][2], 
                    float *particles_tmp, int i){
    const float time_step = 0.01;
    const float G = 6.67430e-11f;
    const float min_cul_radius = 0.25;

    Pj: for (int j = 0; j < (INPUT_LENGTH * 5); j += 5)
    {
        //#pragma HLS pipeline 

        // read particle j
        float xj = particles_tmp[j];
        float yj = particles_tmp[j + 1];
        float massj = particles_tmp[j + 4];
        
        
        BATCH_FORCE: for (int b = 0; b < BATCH_SIZE; b++){
            #pragma HLS unroll
            // Calculate the distance between the two particles in 2D
            // BufP[b][0] = xi, BufP[b][1] = yi, BufP[b][4] = massi
            // BufF[b][0] = force_x, BufF[b][1] = force_y
            if (i + b * 5 == j){continue;}

            float dx = xj - BufP[b][0];
            float dy = yj - BufP[b][1];
            // fixed_t distance = static_cast<fixed_t>(sqrt(static_cast<float>(dx * dx + dy * dy)));
            float distance = sqrt(dx * dx + dy * dy);
            // float distance = 3.0f;
            //  Define gravitational constant
            //  Calculate the gravitational force in 2D

            if (distance <= min_cul_radius)
            {
                float force_magnitude = (G * BufP[b][4] * massj) / (distance * distance);
                // Calculate force components in 2D
                BufF[b][0] += force_magnitude * (dx / distance); //dependency in accumulation
                BufF[b][1] += force_magnitude * (dy / distance); //dependency in accumulation
            }
        }
            
    }
}