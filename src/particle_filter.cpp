/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <float.h>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of particles
	num_particles = 100;

	// Adjust particles and wieghts
	particles.resize(num_particles);
	weights.resize(num_particles);

	// Normal (Gaussian) distribution for x, y, and theta
	normal_distribution<double> distX(x, std[0]);
	normal_distribution<double> distY(y, std[1]);
	normal_distribution<double> distTheta(theta, std[2]);

	// Engine to generate random particles
	random_device rd;
  default_random_engine gen(rd());

	for (int i = 0; i < num_particles; i++) {

		particles[i].id = i;
		particles[i].x = distX(gen);
		particles[i].y = distY(gen);
		particles[i].theta = distTheta(gen);
		particles[i].weight = 1.0;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Normal (Gaussian) distribution for x, y, and theta noise
	default_random_engine gen;
	normal_distribution<double> distX(0, std_pos[0]);
	normal_distribution<double> distY(0, std_pos[1]);
	normal_distribution<double> distTheta(0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {

		if (yaw_rate == 0) {
			particles[i].x += velocity * cos(particles[i].theta) * delta_t;
			particles[i].y += velocity * sin(particles[i].theta) * delta_t;
		} else {
			particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Add noise
		particles[i].x += distX(gen);
		particles[i].y += distY(gen);
		particles[i].theta += distTheta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// I honestly don't see how implementing this will help updateWeights. Is the predicted in the same coordinate
	// system as observations? If so then that means I have to convert to map's coordinate in updateWeights, which will
	// result in creating a new observations vector for every particle.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// Pre-compute constants for the mult-variate Gaussian distribution
	const double mvGauNorm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	const double xDenom = 2 * std_landmark[0] * std_landmark[0];
	const double yDenom = 2 * std_landmark[1] * std_landmark[1];
	
	// Declare variables that will used later
	vector<Map::single_landmark_s> landmarkList = map_landmarks.landmark_list;
	double mvGauss;
	double transformedX;
	double transformedY;
	vector<double> distToLandmark (landmarkList.size());
	double particleDistToLandmark;
	int minPos;
	double deltaX;
	double deltaY;

	for (int i = 0; i < num_particles; i++) {

		// Has to be initialised for each particle
		mvGauss = 1.0;

		for (int j = 0; j < observations.size(); j++) {

			// Transform each observation to map coordinate system
			transformedX = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
			transformedY = particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;

			for (int k = 0; k < landmarkList.size(); k++) {

				// Only calculate distanses for landmarks within sersor range
				particleDistToLandmark = sqrt(pow(particles[i].x - landmarkList[k].x_f, 2) + pow(particles[i].y - landmarkList[k].y_f, 2));
				distToLandmark[k] = particleDistToLandmark <= sensor_range 
					? sqrt(pow(transformedX - landmarkList[k].x_f, 2) + pow(transformedY - landmarkList[k].y_f, 2))
					: DBL_MAX; // to override the default value of zero
			}

			// Find the closest landmark
			minPos = distance(
				distToLandmark.begin(),
				min_element(distToLandmark.begin(), distToLandmark.end())
			);

			deltaX = landmarkList[minPos].x_f - transformedX;
			deltaY = landmarkList[minPos].y_f - transformedY;

			// Calculate mult-variate Gaussian distribution
			mvGauss *= mvGauNorm * exp(-(pow(deltaX, 2)/xDenom + pow(deltaY, 2)/yDenom));
		}
		
		// Update the particle wight
		particles[i].weight = mvGauss;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> sampledParticles(num_particles);
	random_device rd;
  default_random_engine gen(rd());
	
	for (int i = 0; i < num_particles; i++) {

		// Sample particles by wight
		discrete_distribution<int> randIndex(weights.begin(), weights.end());
		sampledParticles[i] = particles[randIndex(gen)];
	}

	particles = sampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
		return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
