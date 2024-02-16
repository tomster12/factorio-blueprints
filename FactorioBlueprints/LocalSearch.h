#pragma once

#include <vector>

namespace ls
{
	class State;
	using StatePtr = std::shared_ptr<State>;

	class State
	{
	public:
		virtual float getCost() = 0;
		virtual std::vector<StatePtr> getNeighbors() = 0;
	};

	StatePtr hillClimbing(StatePtr start, int maxIterations = 100)
	{
		std::vector<StatePtr> cached{ start };
		StatePtr current = start;

		for (size_t it = 0; it < maxIterations; it++)
		{
			// Find best neighbour
			StatePtr best = current;
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
