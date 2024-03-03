#pragma once

#include <vector>

namespace ls
{
	template<typename T>
	class State
	{
	public:
		static size_t cachedSize() { return cachedStates.size(); }

		static std::shared_ptr<T> getCached(std::shared_ptr<T> state)
		{
			const auto ptrEq = [&](std::shared_ptr<T>& other) { return *other == *state; };
			auto it = std::find_if(cachedStates.begin(), cachedStates.end(), ptrEq);
			if (it == cachedStates.end())
			{
				cachedStates.push_back(state);
				return state;
			}
			else
			{
				return *it;
			}
		}

		static void clearCache()
		{
			cachedStates.clear();
		}

		virtual float getCost() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbors() = 0;
		virtual bool operator==(T& other) const = 0;

	protected:
		static std::vector<std::shared_ptr<T>> cachedStates;
	};

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations = 100, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);

		// Until max iterations or local minimum
		std::shared_ptr<T> current = start;
		for (size_t it = 0; it < maxIterations; it++)
		{
			if (toLog) std::cout << "Iteration " << it << ": cost = " << current->getCost() << ", " << current->getNeighbors().size() << " neighbours." << std::endl;

			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbor : current->getNeighbors())
			{
				if (neighbor->getCost() < best->getCost()) best = neighbor;
			}

			// Found local minimum
			if (best == current)
			{
				if (toLog) std::cout << "Local minimum found." << std::endl;
				break;
			}

			// Move to best neighbour
			if (toLog) std::cout << "Best: " << best->getCost() << std::endl;
			current = best;
		}

		if (toLog) std::cout << "Hill Climbing: cost=" << current->getCost() << ", statesEvaluated=" << T::cachedSize() << std::endl << std::endl;
		return current;
	}

	template<typename T>
	std::shared_ptr<T> simulatedAnnealing(std::shared_ptr<T> start, float temperature = 1000, float coolingRate = 0.005, int maxIterations = 100, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);

		// Until max iterations or local minimum
		std::shared_ptr<T> current = start;
		for (size_t it = 0; it < maxIterations && temperature > 0.01f; it++)
		{
			if (toLog) std::cout << "\nIteration=" << it << ", temp=" << temperature << ": cost=" << current->getCost() << ", neighbours=" << current->getNeighbors().size() << "." << std::endl;

			// Find random neighbour
			std::shared_ptr<T> next = current->getNeighbors()[rand() % current->getNeighbors().size()];
			float delta = next->getCost() - current->getCost();

			// If better, move to neighbour
			if (delta <= 0)
			{
				current = next;
				if (toLog) std::cout << "Moved to better neighbour" << std::endl;
			}

			// If worse, accept with probability
			else
			{
				float acceptanceProbability = exp(-delta / temperature);
				if (toLog) std::cout << "Delta=" << delta << ", Acceptance=" << acceptanceProbability << " ";
				if ((acceptanceProbability > (rand() / (float)RAND_MAX)))
				{
					current = next;
					if (toLog) std::cout << "Moved to worse neighbour.";
				}
				if (toLog) std::cout << std::endl;
			}

			// Cool system
			temperature *= 1 - coolingRate;
		}

		if (toLog) std::cout << "Annealing: cost=" << current->getCost() << ", statesEvaluated=" << T::cachedSize() << std::endl << std::endl;
		return current;
	}
}
