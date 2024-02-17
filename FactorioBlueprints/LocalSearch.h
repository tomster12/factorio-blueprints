#pragma once

#include <vector>

namespace ls
{
	template<typename T>
	class State
	{
	public:
		virtual float getCost() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbors() = 0;
		virtual bool operator==(const T& other) const = 0;
	};

	template<typename T>
	bool operator==(const State<T>& lhs, const State<T>& rhs) { return *lhs == *rhs; }

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations = 100)
	{
		std::vector<std::shared_ptr<T>> cached{ start };
		std::shared_ptr<T> current = start;

		for (size_t it = 0; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (auto& neighbor : current->getNeighbors())
			{
				if (std::find(cached.begin(), cached.end(), neighbor) != cached.end()) continue;
				if (neighbor->getCost() < best->getCost()) best = neighbor;
				cached.push_back(neighbor);
			}

			// Found local minimum
			if (best == current) break;

			// Move to best neighbour
			current = best;
		}

		return current;
	}
}
