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
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations = 100)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		std::vector<std::shared_ptr<T>> cached{ start };
		std::shared_ptr<T> current = start;

		for (size_t it = 0; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbor : current->getNeighbors())
			{
				// If cached do not re-evaluate
				const auto ptrEq = [&](std::shared_ptr<T>& other) { return *other == *neighbor; };
				if (std::find_if(cached.begin(), cached.end(), ptrEq) != cached.end()) continue;
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
}
