#pragma once

#include <vector>

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

		static void clearCache() { cachedStates.clear(); }

		static size_t getCacheSize() { return cachedStates.size(); }

		bool operator==(State<T>& other) const { return hash == hash; }

		virtual float getCost() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbors() = 0;

	protected:
		static std::map<size_t, std::shared_ptr<T>> cachedStates;
		size_t hash = 0;
	};

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations = 100, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);

		// Until max iterations or local minimum
		std::shared_ptr<T> current = start;
		std::shared_ptr<T> best = start;

		if (toLog) std::cout << "Start, cost: " << current->getCost() << std::endl;

		size_t it = 0;
		for (; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbor : current->getNeighbors())
			{
				if (neighbor->getCost() < best->getCost())
				{
					best = neighbor;
				}
			}

			// Found local minimum
			if (best == current)
			{
				if (toLog) std::cout << "It " << it << ", local minimum" << std::endl;
				break;
			}

			// Move to best neighbour
			if (toLog) std::cout << "It " << it << ", better cost: " << best->getCost() << std::endl;
			current = best;

			// Update best
			if (current->getCost() < best->getCost()) best = current;
		}

		if (toLog) std::cout << "Finished " << it << " iterations, cost: " << best->getCost() << ", states evaluated: " << T::getCacheSize() << std::endl << std::endl;
		return best;
	}

	template<typename T>
	std::shared_ptr<T> simulatedAnnealing(std::shared_ptr<T> start, float temperature = 1000, float coolingRate = 0.005, int maxIterations = 100, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);

		// Until max iterations or local minimum
		std::shared_ptr<T> current = start;
		std::shared_ptr<T> best = start;

		if (toLog) std::cout << "Start, cost: " << current->getCost() << std::endl;

		size_t it = 0;
		for (; it < maxIterations && temperature > 0.01f; it++)
		{
			// Find random neighbour
			std::shared_ptr<T> next = current->getNeighbors()[rand() % current->getNeighbors().size()];
			float delta = next->getCost() - current->getCost();

			// If better, move to neighbour
			if (delta <= 0)
			{
				current = next;
				if (toLog) std::cout << "It " << it << ", better cost: " << next->getCost() << std::endl;
			}

			// If worse, accept with probability
			else
			{
				float acceptanceProbability = exp(-delta / temperature);
				if ((acceptanceProbability > (rand() / (float)RAND_MAX)))
				{
					current = next;
					if (toLog) std::cout << "It " << it << ", worse cost: " << next->getCost() << ", chance " << acceptanceProbability << std::endl;
				}
			}

			// Cool system
			temperature *= 1 - coolingRate;

			// Update best
			if (current->getCost() < best->getCost()) best = current;
		}

		if (toLog) std::cout << "Finished " << it << " iterations, cost: " << best->getCost() << ", states evaluated: " << T::getCacheSize() << std::endl << std::endl;
		return best;
	}
}
