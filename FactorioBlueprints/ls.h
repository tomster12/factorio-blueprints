#pragma once

#include <map>
#include <vector>
#include "macros.h"
#include "log.h"

namespace ls
{
	template<typename T>
	class StateCache;

	template<typename T>
	class State
	{
	public:
		virtual float getFitness() = 0;
		virtual std::vector<T*> getNeighbours(StateCache<T>& cache) = 0;
		virtual void clearNeighbours() = 0;
		bool operator==(State<T>& other) { return hash == other.hash; }
		size_t getHash() { return hash; }

	protected:
		size_t hash = 0;
	};

	template<typename T>
	class StateCache
	{
	public:
		StateCache()
		{
			cache = std::map<size_t, T*>();
		}

		~StateCache()
		{
			for (auto& pair : cache) delete pair.second;
		}

		T* getCached(T* state)
		{
			size_t hash = state->getHash();
			if (cache.find(hash) == cache.end()) cache[hash] = state;
			else if (state != cache[hash]) delete state;
			return cache[hash];
		}

		void removeCached(T* state)
		{
			size_t hash = state->getHash();
			if (cache.find(hash) != cache.end()) cache.erase(hash);
		}

		size_t getCacheSize()
		{
			return cache.size();
		}

	private:
		std::map<size_t, T*> cache;
	};

	template<typename T>
	T* hillClimbing(T* start, int maxIterations)
	{
		// Initialize with start
		StateCache<T> cache;
		T* current = cache.getCached(start);
		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");

		// Loop until max iterations or local maximum
		size_t it = 0;
		for (; it < maxIterations; it++)
		{
			// Find best neighbour
			std::vector<T*> neighbours = current->getNeighbours(cache);

			T* best = current;
			for (T* neighbour : neighbours)
			{
				if (neighbour->getFitness() > best->getFitness()) best = neighbour;
			}

			// Found local maximum
			if (current == best)
			{
				LOG(LOCAL_SEARCH, "It " << it << ", local maximum\n");
				break;
			}

			// Move to best neighbour
			current = best;
			LOG(LOCAL_SEARCH, "It " << it << ", better fitness: " << best->getFitness() << "\n");
		}

		// Detach current from cache, then delete all cached states
		current->clearNeighbours();
		cache.removeCached(current);

		// Return best state
		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << current->getFitness() << ", states evaluated: " << cache.getCacheSize() << "\n\n");
		return current;
	}

	template<typename T>
	T* simulatedAnnealing(T* start, float temperature, float coolingRate, int maxIterations)
	{
		// Initialize with start
		StateCache<T> cache;
		T* current = cache.getCached(start);
		LOG(LOCAL_SEARCH, "Start, fitness: " << current->getFitness() << "\n");

		// Loop until max iterations or local maximum
		size_t it = 0;
		T* best = start;
		for (; it < maxIterations && temperature > 0.01f; it++)
		{
			// Find random neighbour
			std::vector<T*> neighbours = current->getNeighbours(cache);
			T* next = neighbours[rand() % neighbours.size()];
			next = cache.getCached(next);

			// If better, move to neighbour
			float delta = next->getFitness() - current->getFitness();
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

		// Detach current from cache, then delete all cached states
		best->clearNeighbours();
		cache.removeCached(best);

		// Return best state
		LOG(LOCAL_SEARCH, "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states seen: " << cache.getCacheSize() << "\n\n");
		return best;
	}
}
