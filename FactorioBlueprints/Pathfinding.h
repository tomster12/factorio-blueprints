#pragma once

#include <algorithm>

namespace pf
{
	float EuclideanDistance(float x1, float y1, float x2, float y2)
	{
		return static_cast<float>(sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
	}

	template<typename T>
	class State
	{
	public:
		virtual bool operator==(T& other) const = 0;
		virtual float getCost(std::shared_ptr<T> target) = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbours() = 0;
		virtual std::shared_ptr<T> getParent() const { return parent; }

	protected:
		State(std::shared_ptr<T> parent = nullptr) : parent(parent) {}
		std::shared_ptr<T> parent;
	};

	template<typename T>
	std::vector<std::shared_ptr<T>> asPathfinding(std::shared_ptr<T> start, std::shared_ptr<T> goal, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		// A* Pathfinding Algorithm
		std::vector<std::shared_ptr<T>> path;
		std::set<std::shared_ptr<T>> closedSet;
		std::set<std::shared_ptr<T>> openSet;
		openSet.insert(start);

		// Until open set is empty
		while (!openSet.empty())
		{
			// Get node with lowest f score
			std::shared_ptr<T> current = *openSet.begin();
			for (const auto& node : openSet)
			{
				if (node->getCost(goal) < current->getCost(goal))
				{
					current = node;
				}
			}

			// Found goal
			if (*current == *goal)
			{
				std::shared_ptr<T> node = current;
				while (node != nullptr)
				{
					path.push_back(node);
					node = node->getParent();
				}
				std::reverse(path.begin(), path.end());
				return path;
			}

			// Remove current from open set and add to closed set
			openSet.erase(current);
			closedSet.insert(current);

			// For each neighbor
			for (std::shared_ptr<T> neighbor : current->getNeighbours())
			{
				// Skip if neighbor is in closed set
				// Find needs to dereference the shared_ptr
				if (std::find_if(closedSet.begin(), closedSet.end(), [&neighbor](const std::shared_ptr<T>& node) { return *node == *neighbor; }) != closedSet.end()) continue;

				// Add neighbor to open set if not already there
				if (std::find_if(openSet.begin(), openSet.end(), [&neighbor](const std::shared_ptr<T>& node) { return *node == *neighbor; }) == openSet.end()) openSet.insert(neighbor);
			}
		}

		return path;
	}
}
