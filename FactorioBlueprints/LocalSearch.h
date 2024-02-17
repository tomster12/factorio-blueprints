#pragma once

#include <vector>

namespace ls
{
	class State;
	using StatePtr = std::shared_ptr<State>;

	class State
	{
	public:
		State(int uid) : uid(uid) {}
		virtual float getCost() = 0;
		virtual std::vector<StatePtr> getNeighbors() = 0;

		bool operator==(const State& other) const { return uid == other.uid; }

	private:
		int uid;
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
				if (std::find_if(cached.begin(), cached.end(), [&](const std::shared_ptr<State>& ptr) { return *ptr == *neighbor; }) != cached.end()) continue;
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
