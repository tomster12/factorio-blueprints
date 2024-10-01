#pragma once

#ifndef LOG
#define LOG(level, ...)  \
	if (LOG_##level##_ENABLED) { \
		std::cout << __VA_ARGS__; \
	}
#endif

#include <vector>
#include "DataLogger.h"

namespace ls
{
	template<typename T>
	class State
	{
	public:
		static std::shared_ptr<T> getCached(std::shared_ptr<T> state)
		{
			size_t hash = state->hash;
			if (cachedStates.find(hash) == cachedStates.end()) cachedStates[hash] = state;
			return cachedStates[hash];
		}

		static void clearCache() { cachedStates = std::map<size_t, std::shared_ptr<T>>(); }

		static size_t getCacheSize() { return cachedStates.size(); }

		bool operator==(State<T>& other) const { return hash == hash; }

		virtual float getFitness() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbours() = 0;
		virtual std::vector<float> generateDataLog() = 0;

	protected:
		static std::map<size_t, std::shared_ptr<T>> cachedStates;
		size_t hash = 0;
	};

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations, std::shared_ptr<DataLogger> dataLogger)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		// Initialize with start state
		T::clearCache();
		T::getCached(start);
		std::shared_ptr<T> current = start;

		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");
		dataLogger->log(current->generateDataLog());

		// Until max iterations or local maximum
		std::shared_ptr<T> best = start;
		size_t it = 0;
		for (; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbour : current->getNeighbours())
			{
				if (neighbour->getFitness() > best->getFitness())
				{
					best = neighbour;
				}
			}

			// Found local maximum
			if (best == current)
			{
				LOG(LOCAL_SEARCH, "It " << it << ", local maximum\n");
				break;
			}

			// Move to best neighbour
			LOG(LOCAL_SEARCH, "It " << it << ", better fitness: " << best->getFitness() << "\n");
			dataLogger->log(best->generateDataLog());
			current = best;

			// Update best
			if (current->getFitness() > best->getFitness()) best = current;
		}

		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states evaluated: " << T::getCacheSize() << "\n\n");
		return best;
	}

	template<typename T>
	std::shared_ptr<T> simulatedAnnealing(std::shared_ptr<T> start, float temperature, float coolingRate, int maxIterations, std::shared_ptr<DataLogger> dataLogger)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);
		std::shared_ptr<T> current = start;

		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");
		dataLogger->log(current->generateDataLog());

		// Until max iterations or local maximum
		size_t it = 0;
		std::shared_ptr<T> best = start;
		for (; it < maxIterations && temperature > 0.01f; it++)
		{
			// Find random neighbour
			std::shared_ptr<T> next = current->getNeighbours()[rand() % current->getNeighbours().size()];
			float delta = next->getFitness() - current->getFitness();

			// If better, move to neighbour
			if (delta >= 0)
			{
				current = next;
				LOG(LOCAL_SEARCH, "It " << it << ", temperature: " << temperature << ", moving to a better fitness: " << next->getFitness() << "\n");
				dataLogger->log(current->generateDataLog());
			}

			// If worse, accept with probability
			else
			{
				float acceptanceProbability = exp(delta / temperature);
				if ((acceptanceProbability > (rand() / (float)RAND_MAX)))
				{
					current = next;
					LOG(LOCAL_SEARCH, "It " << it << ", temperature: " << temperature << ", moving to a worse fitness: " << next->getFitness() << ", chance " << acceptanceProbability << "\n");
					dataLogger->log(current->generateDataLog());
				}
			}

			// Cool system temperature
			temperature *= 1 - coolingRate;

			// Update best
			if (current->getFitness() > best->getFitness()) best = current;
		}

		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states seen: " << T::getCacheSize() << "\n\n");
		return best;
	}
}
