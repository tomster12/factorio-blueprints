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
		virtual bool operator==(T& other) const = 0;
	};

	template<typename T>
	bool operator==(std::shared_ptr<State<T>>& lhs, std::shared_ptr<State<T>>& rhs)
	{
		//return (*lhs) == (*rhs);
		return (*std::static_pointer_cast<T>(lhs)) == (*std::static_pointer_cast<T>(rhs));
	}

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<State<T>> start, int maxIterations = 100)
	{
		std::vector<std::shared_ptr<State<T>>> cached{ start };
		std::shared_ptr<State<T>> current = start;

		for (size_t it = 0; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<State<T>> best = current;
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

		return std::static_pointer_cast<T>(current);
	}
}
