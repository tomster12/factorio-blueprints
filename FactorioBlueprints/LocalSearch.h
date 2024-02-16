#pragma once

#include <vector>

template <typename T>
class LSState
{
public:
	virtual float getCost() = 0;
	virtual std::vector<std::shared_ptr<T>> getNeighbors() = 0;
};

class LocalSearch
{
public:

	template <typename T>
	static std::shared_ptr<T> hillClimbing(std::shared_ptr<T>&& start, int maxIterations = 100)
	{
		std::vector<std::shared_ptr<T>> cached{ start };
		std::shared_ptr<T> current = start;

		for (size_t it = 0; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			std::vector<std::shared_ptr<T>> neighbours = current->getNeighbors();
			for (std::shared_ptr<T>& neighbor : neighbours)
			{
				// Skip cached neighbours
				if (std::find(cached.begin(), cached.end(), neighbor) != cached.end()) continue;
				cached.push_back(neighbor);

				if (neighbor->getCost() < best->getCost()) best = neighbor;
			}

			// Found local minimum
			if (best == current) break;

			// Move to best neighbour
			current = best;
		}

		return current;
	}
};
