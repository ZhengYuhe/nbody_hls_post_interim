/*
# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: X11
*/

#include "nbody.h"

// ********************************************************
int main()
{
    printf("Starting Nbody sims....\n");
    float *particles;
    float *original_particles;
    float *temp;
    float a;
    float b;
    float c;
    float d;
    float e;

    particles = new float[INPUT_LENGTH * 5];
    if (particles == NULL)
    {
        printf("Error allocating particles\n");
        exit(EXIT_FAILURE);
    }

    original_particles = new float[INPUT_LENGTH * 5];
    if (original_particles == NULL)
    {
        printf("Error allocating original_particles\n");
        exit(EXIT_FAILURE);
    }

    temp = new float[INPUT_LENGTH * 5];
    if (temp == NULL)
    {
        printf("Error allocating temp\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < INPUT_LENGTH * 5; i += 5)
    {
        a = 0.1f * i;
        b = 0.2f * i;
        c = 0.3f * i;
        d = 0.4f * i;
        e = 0.5f * i;

        particles[i] = a;
        particles[i + 1] = b;
        particles[i + 2] = c;
        particles[i + 3] = d;
        particles[i + 4] = e;

        original_particles[i] = a;
        original_particles[i + 1] = b;
        original_particles[i + 2] = c;
        original_particles[i + 3] = d;
        original_particles[i + 4] = e;
    }

    printf("Nbody Initialised particles!\n");
    krnl_nbody(particles, temp, ITERATIONS);

    bool passed = false;
    for (int i = 0; i < INPUT_LENGTH * 5; i++)
    {
        if (original_particles[i] != particles[i])
        {
            passed = true;
        }
    }

    delete[] particles;
    delete[] original_particles;
    delete[] temp;

    if (passed)
    {
        printf("Nbody ran through successfully!\n");
    }
    else
    {
        printf("Nbody failed!\n");
    }

    return 0;
}