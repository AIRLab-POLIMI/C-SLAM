/*
 * NoiseGeneration.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: davide
 */

#include "NoiseGeneration.h"

#include <random>

void add_gaussian_noise(double *to, unsigned int size, double mean,
    double std) {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::normal_distribution<double> dist(mean, std);

  for (int k = 0; k < size; ++k) {
    to[k] += dist(gen);
  }
}

