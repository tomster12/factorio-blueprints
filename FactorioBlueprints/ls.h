#pragma once

#include <vector>
#include <map>
#include <iostream>
#include "macros.h"
#include "log.h"

namespace ls
{
	template<typename T>
	class State
	{
	public:
		bool operator==(State<T>& other) const { return getHash() == other.getHash(); }
		virtual float getFitness() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbours() = 0;
		virtual size_t getHash() = 0;
		virtual std::vector<float> generateDataLog() = 0;
	};

	template<typename T>
	class StateCache
	{
	public:
		inline StateCache() { cache = std::map<size_t, std::shared_ptr<T>>(); }
		inline std::shared_ptr<T> getCached(std::shared_ptr<T> state)
		{
			size_t hash = state->getHash();
			if (cache.find(hash) == cache.end()) cache[hash] = state;
			return cache[hash];
		}
		inline size_t getCacheSize() { return cache.size(); }

	private:
		std::map<size_t, std::shared_ptr<T>> cache;
	};

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		// Initialize with start state
		StateCache<T> cache;
		std::shared_ptr<T> current = cache.getCached(start);
		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");

		// Until max iterations or local maximum
		std::shared_ptr<T> best = start;
		size_t it = 0;
		for (; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbour : current->getNeighbours())
			{
				std::shared_ptr<T> cached = cache.getCached(neighbour);
				if (cached->getFitness() > best->getFitness())
				{
					best = cached;
				}
			}

			// Found local maximum
			if (best == current)
			{
				LOG(LOCAL_SEARCH, "It " << it << ", local maximum\n");
				break;
			}

			// Move to best neighbour
			current = best;
			LOG(LOCAL_SEARCH, "It " << it << ", better fitness: " << best->getFitness() << "\n");

			// Update current best
			if (current->getFitness() > best->getFitness()) best = current;
		}

		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states evaluated: " << cache.getCacheSize() << "\n\n");
		return best;
	}

	template<typename T>
	std::shared_ptr<T> simulatedAnnealing(std::shared_ptr<T> start, float temperature, float coolingRate, int maxIterations)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		// Initialize with start state
		StateCache<T> cache;
		std::shared_ptr<T> current = cache.getCached(start);
		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");

		// Until max iterations or local maximum
		size_t it = 0;
		std::shared_ptr<T> best = start;
		for (; it < maxIterations && temperature > 0.01f; it++)
		{
			// Find random neighbour
			std::shared_ptr<T> next = current->getNeighbours()[rand() % current->getNeighbours().size()];
			next = cache.getCached(next);
			float delta = next->getFitness() - current->getFitness();

			// If better, move to neighbour
			if (delta >= 0)
			{
				current = next;
				LOG(LOCAL_SEARCH, "It " << it << ", temperature: " << temperature << ", moving to a better fitness: " << next->getFitness() << "\n");
			}

			// If worse, accept with probability
			else
			{
				float acceptanceProbability = exp(delta / temperature);
				if ((acceptanceProbability > (rand() / (float)RAND_MAX)))
				{
					current = next;
					LOG(LOCAL_SEARCH, "It " << it << ", temperature: " << temperature << ", moving to a worse fitness: " << next->getFitness() << ", chance " << acceptanceProbability << "\n");
				}
			}

			// Cool system temperature
			temperature *= 1 - coolingRate;

			// Update best
			if (current->getFitness() > best->getFitness()) best = current;
		}

		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states seen: " << cache.getCacheSize() << "\n\n");
		return best;
	}
}
